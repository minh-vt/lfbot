#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace lf::drive {

// Chế độ nhiệm vụ (dùng chung sim + esp32).
enum class RunMode : std::uint8_t { kIdle = 0, kRun1 = 1, kRun2 = 2, kDone = 3 };

// Pha né vật cản (ticket 36): phanh → cam kết bên → vòng → bám lại line.
enum class AvoidPhase : std::uint8_t { kIdle = 0, kBrake, kCommit, kRejoin };

// Đọc cảm biến không-thứ-nguyên mỗi tick.
struct Readings {
  float line_offset = 0.0f;      // [-1, 1]: lệch vạch chuẩn hóa (dương = vạch bên phải)
  float line_certainty = 0.0f;   // [0, 1]: độ tin cậy vạch (liên tục, không phải bool)
  float heading_rate = 0.0f;     // [rad/s] gyro; 0 = không có IMU → fallback de/ds (ticket 37)
  bool finish_detected = false;  // cảm biến vạch đích
  float obstacle_danger = 0.0f;  // [0, 1]
  float obstacle_side = 0.0f;    // [-1, 1]: âm = vật cản bên trái
  std::int32_t enc_l = 0;        // tick thô
  std::int32_t enc_r = 0;        // tick thô
  float dt = 0.002f;             // giây
};

// Đầu ra không-thứ-nguyên.
struct DriveCommand {
  float left_duty = 0.0f;   // [-1, 1]
  float right_duty = 0.0f;  // [-1, 1]
  bool brake = false;
};

// Ký ức nóng của trí tuệ (nhỏ, value-returned).
struct DriveState {
  float line_estimate = 0.0f;       // lệch ngang ước lượng (liên tục)
  float prev_line_estimate = 0.0f;  // cho de/ds
  float heading = 0.0f;             // ψ ước lượng (ticket 37)
  float speed = 0.0f;               // tốc độ (tick/s) đã lọc — gain-scheduling (ticket 33)
  float prev_tick = 0.0f;           // mốc quãng đường tick trước
  float run1_speed = 0.0f;          // error-budget speed Run 1 (ticket 35)
  bool finish_latched = false;
  AvoidPhase avoid = AvoidPhase::kIdle;  // state machine né (ticket 36)
  float avoid_side = 0.0f;               // ±1: bên đã cam kết
};

// Dung lượng tối đa của plan: 4096 tick × 20mm = 81.9m (thừa cho sa bàn ≤ 41m).
inline constexpr std::size_t kMaxPlanTicks = 4096;

// Cầu nối 1-D Run1 → Run2. Fixed-capacity: không cấp phát động trong vòng 500Hz.
struct RacePlan {
  std::array<float, kMaxPlanTicks> planned_steer{};    // ghi Run 1 (tín hiệu lái đã thực thi)
  std::array<float, kMaxPlanTicks> line_curve{};       // ghi Run 1: độ cong thuần (geometry)
  std::array<float, kMaxPlanTicks> obstacle_side{};    // ghi Run 1 (±1), giãn khi plan_speed
  std::array<float, kMaxPlanTicks> speed_limit{};      // plan_speed sau Run 1 (backward-pass)
  std::array<float, kMaxPlanTicks> offset_setpoint{};  // d[t]: racing-line offset (ticket 25)
  std::size_t length = 0;
  bool ready = false;  // true sau plan_speed() → step dùng feedforward (Run 2)

  void reset() {
    length = 0;
    ready = false;
  }
  [[nodiscard]] float planned_steer_at(float tick) const;
  [[nodiscard]] float line_curve_at(float tick) const;
  [[nodiscard]] float obstacle_at(float tick) const;
  [[nodiscard]] float speed_limit_at(float tick) const;
  [[nodiscard]] float offset_at(float tick) const;
};

// Bảng slip-boundary ĐO (ticket 34). Trục `curve` cùng đơn vị với `line_curve` ∈ [0,1].
// Số điểm cố định, nội suy tuyến tính O(n) — không cấp phát trong vòng 500Hz.
inline constexpr std::size_t kSpeedLimitPoints = 16;
struct SpeedLimitTable {
  std::array<float, kSpeedLimitPoints> curve{};
  std::array<float, kSpeedLimitPoints> limit{};
  std::size_t count = 0;

  void add(float c, float v) {
    if (count >= kSpeedLimitPoints) {
      return;
    }
    curve[count] = c;
    limit[count] = v;
    ++count;
  }
  [[nodiscard]] float lookup(float c) const {
    if (count == 0) {
      return 1.0f;
    }
    if (c <= curve[0]) {
      return limit[0];
    }
    if (c >= curve[count - 1]) {
      return limit[count - 1];
    }
    for (std::size_t i = 1; i < count; ++i) {
      if (c <= curve[i]) {
        const float t = (c - curve[i - 1]) / (curve[i] - curve[i - 1]);
        return limit[i - 1] * (1.0f - t) + limit[i] * t;
      }
    }
    return limit[count - 1];
  }
};

// Fallback thận trọng CHƯA calibration (ticket 34): sim/on-robot thay bằng bảng đo slip-boundary.
[[nodiscard]] inline SpeedLimitTable make_default_speed_limit_table() {
  SpeedLimitTable table;
  table.add(0.00f, 1.00f);
  table.add(0.40f, 0.70f);
  table.add(0.80f, 0.45f);
  table.add(1.00f, 0.30f);
  return table;
}

// Tầng 2 — chân lý robot: hình học (odometry) + hằng đo một lần. KHÔNG có gain chọn tay.
struct RobotTuning {
  // Hình học trong đơn vị tick (odometry truth) — SUY, không tune.
  float geometry_l = 6.9f;  // wheelbase / tick-length (138mm / 20mm)
  float geometry_s = 2.1f;  // sensor half-span / tick-length (42mm / 20mm)
  // Vòng kín (ĐO một lần).
  float bandwidth = 4.0f;  // ωn [rad/s] — motor bandwidth (N20 ~30-50ms → 3-6 rad/s)
  float damping = 0.7f;    // ζ (chuẩn, không tune)
  // Envelope slip-boundary (ĐO, ticket 34).
  SpeedLimitTable speed_limit_table = make_default_speed_limit_table();

  [[nodiscard]] float speed_limit(float curve) const {
    return speed_limit_table.lookup(std::abs(curve));
  }
};

// Tầng 3 — tham số phụ thuộc sa bàn.
struct TrackTuning {
  float brake_limit = 0.02f;          // giảm ga tối đa mỗi tick (backward-pass)
  std::int32_t obstacle_window = 20;  // cửa sổ giãn obstacle_side (tick)
  float sprint_margin = 0.9f;         // biên an toàn Run 2 (chạy dưới envelope đo)
};

// Cấu hình đầy đủ truyền vào step(): robot (measure-once) + track (trim-on-track).
struct Tuning {
  RobotTuning robot;
  TrackTuning track;
};

}
