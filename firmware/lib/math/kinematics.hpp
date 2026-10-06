#pragma once

namespace lf {

struct Twist2D {
  float linear_v_mps = 0.0f;
  float angular_w_radps = 0.0f;
};

class DifferentialDriveKinematics {
public:
  explicit DifferentialDriveKinematics(float wheel_base_m);

  [[nodiscard]] Twist2D forward(float v_left_mps, float v_right_mps) const;

  // Chuyển đổi và chuẩn hóa vận tốc bánh xe sang tỷ lệ xung PWM [-1.0, 1.0] (Poka-Yoke)
  static float velocity_to_pwm(float wheel_v_mps, float max_v_mps);

private:
  float wheel_base_m_ = 0.115f;
};

}  // namespace lf
