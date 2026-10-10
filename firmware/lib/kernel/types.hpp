#pragma once

#include <cstdint>
#include <vector>

namespace lf::kernel {

// Đầu vào không-thứ-nguyên mỗi tick.
struct SensorFrame {
  float line_error = 0.0f;       // [-1, 1]: lệch vạch chuẩn hóa (dương = vạch bên phải)
  bool line_detected = false;    // có thấy vạch không
  bool finish_detected = false;  // cảm biến vạch đích
  float obstacle_danger = 0.0f;  // [0, 1]
  float obstacle_side = 0.0f;    // [-1, 1]: âm = vật cản bên trái
  std::int32_t enc_l = 0;        // tick thô bánh trái
  std::int32_t enc_r = 0;        // tick thô bánh phải
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

// Cầu nối 1-D Run1 → Run2 (lớn, const-ref vào step).
struct TrackPlan {
  std::vector<float> steer_ff;       // ghi Run 1 (steering theo vạch, không né)
  std::vector<float> obstacle_side;  // ghi Run 1 (±1), làm mượt khi bake
  std::vector<float> throttle_max;   // bake sau Run 1 (backward-pass)
};

// Thuần gain, không đơn vị.
struct TuneConfig {
  float steer_kp = 3.0f;
  float steer_kd = 0.15f;
  float d_filter_alpha = 0.5f;
  float base_throttle = 0.5f;
  float corner_slowdown = 0.6f;   // grip proxy: 1 - corner_slowdown*|steer|
  float evasion_bias = 0.4f;      // lệch lái né vật cản
  float evasion_slowdown = 0.7f;  // giảm ga khi danger
  float sprint_scale = 1.2f;
  float decel_per_tick = 0.02f;       // giảm ga tối đa mỗi tick (backward-pass)
  std::int32_t obstacle_window = 20;  // cửa sổ làm mượt obstacle_side (tick)
};

}  // namespace lf::kernel
