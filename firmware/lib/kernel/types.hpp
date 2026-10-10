#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace lf::kernel {

// Chế độ nhiệm vụ (dùng chung sim + esp32).
enum class RunMode : std::uint8_t { kIdle = 0, kRun1 = 1, kRun2 = 2, kDone = 3 };

// Đầu vào không-thứ-nguyên mỗi tick.
struct SensorFrame {
  float line_error = 0.0f;  // [-1, 1]: lệch vạch chuẩn hóa (dương = vạch bên phải)
  bool line_detected = false;
  bool finish_detected = false;  // cảm biến vạch đích
  float obstacle_danger = 0.0f;  // [0, 1]
  float obstacle_side = 0.0f;    // [-1, 1]: âm = vật cản bên trái
  std::int32_t enc_l = 0;        // tick thô
  std::int32_t enc_r = 0;        // tick thô
  float dt = 0.002f;             // giây
};

// Đầu ra không-thứ-nguyên.
struct MotorCommand {
  float left_duty = 0.0f;   // [-1, 1]
  float right_duty = 0.0f;  // [-1, 1]
  bool brake = false;
};

// Ký ức nóng của trí tuệ (nhỏ, value-returned).
struct KernelState {
  float prev_error = 0.0f;
  float filtered_derivative = 0.0f;
  float last_direction = 0.0f;  // ±1: hướng vạch nhớ khi mất vạch
  bool finish_latched = false;
};

// Dung lượng tối đa của plan: 4096 tick × 20mm = 81.9m (thừa cho sa bàn ≤ 41m).
inline constexpr std::size_t kMaxPlanTicks = 4096;

// Cầu nối 1-D Run1 → Run2. Fixed-capacity: không cấp phát động trong vòng 500Hz.
struct TrackPlan {
  std::array<float, kMaxPlanTicks> steer_ff{};       // ghi Run 1 (steering theo vạch, không né)
  std::array<float, kMaxPlanTicks> obstacle_side{};  // ghi Run 1 (±1), giãn khi bake
  std::array<float, kMaxPlanTicks> throttle_max{};   // bake sau Run 1 (backward-pass)
  std::size_t length = 0;
  bool baked = false;  // true sau bake() → step dùng feedforward (Run 2)

  void reset() {
    length = 0;
    baked = false;
  }
  [[nodiscard]] float steer_at(float tick) const;
  [[nodiscard]] float obstacle_at(float tick) const;
  [[nodiscard]] float throttle_at(float tick) const;
};

// Tầng 2 — hằng số robot, đo một lần, không đổi theo sa bàn. Thuần gain.
struct RobotTuning {
  float steer_kp = 3.0f;
  float steer_kd = 0.15f;
  float d_filter_alpha = 0.5f;
  float corner_slowdown = 0.6f;   // grip proxy: 1 - corner_slowdown*|steer|
  float evasion_bias = 0.4f;      // lệch lái né vật cản
  float evasion_slowdown = 0.7f;  // giảm ga khi danger
};

// Tầng 3 — tham số phụ thuộc sa bàn, suy từ Run 1 (xem kernel/tuning.hpp).
struct TrackTuning {
  float base_throttle = 0.5f;
  float sprint_scale = 1.2f;
  float decel_per_tick = 0.02f;       // giảm ga tối đa mỗi tick (backward-pass)
  std::int32_t obstacle_window = 20;  // cửa sổ giãn obstacle_side (tick)
};

// Cấu hình đầy đủ truyền vào step(): robot (measure-once) + track (trim-on-track).
struct TuneConfig {
  RobotTuning robot;
  TrackTuning track;
};

}
