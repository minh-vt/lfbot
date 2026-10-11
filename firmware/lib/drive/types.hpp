#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace lf::drive {

// Chế độ nhiệm vụ (dùng chung sim + esp32).
enum class RunMode : std::uint8_t { kIdle = 0, kRun1 = 1, kRun2 = 2, kDone = 3 };

// Đọc cảm biến không-thứ-nguyên mỗi tick.
struct Readings {
  float line_offset = 0.0f;      // [-1, 1]: lệch vạch chuẩn hóa (dương = vạch bên phải)
  float line_certainty = 0.0f;   // [0, 1]: độ tin cậy vạch (liên tục, không phải bool)
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
  float prev_error = 0.0f;
  float filtered_derivative = 0.0f;
  float line_estimate = 0.0f;  // lệch ngang ước lượng (liên tục, không phải ±1)
  bool finish_latched = false;
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

// Tầng 2 — hằng số robot, đo một lần, không đổi theo sa bàn. Thuần gain.
struct RobotTuning {
  float steer_kp = 3.0f;
  float steer_kd = 0.15f;
  float d_filter_alpha = 0.5f;
  float corner_limit = 0.6f;    // hệ số tuyến tính của speed_limit(κ) (fallback)
  float avoid_strength = 0.4f;  // lệch lái né vật cản
  float avoid_slowdown = 0.7f;  // giảm ga khi danger
  float offset_gain = 0.5f;     // gain racing-line offset (ticket 25, placeholder)

  // Giới hạn tốc độ theo ĐỘ CONG (geometry). Fallback tuyến tính; ticket 23-hardware
  // thay bằng curve đo on-robot `speed_limit(κ)` (index theo κ, không theo steer).
  [[nodiscard]] float speed_limit(float kappa) const {
    return std::clamp(1.0f - corner_limit * std::abs(kappa), 0.0f, 1.0f);
  }
};

// Tầng 3 — tham số phụ thuộc sa bàn, suy từ Run 1.
struct TrackTuning {
  float base_throttle = 0.5f;
  float sprint_scale = 1.2f;
  float brake_limit = 0.02f;          // giảm ga tối đa mỗi tick (backward-pass)
  std::int32_t obstacle_window = 20;  // cửa sổ giãn obstacle_side (tick)
};

// Cấu hình đầy đủ truyền vào step(): robot (measure-once) + track (trim-on-track).
struct Tuning {
  RobotTuning robot;
  TrackTuning track;
};

}
