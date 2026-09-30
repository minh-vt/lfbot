#pragma once

namespace lf {

struct Twist2D {
  float linear_v_mps = 0.0f;
  float angular_w_radps = 0.0f;
};

struct WheelVelocities {
  float left_mps = 0.0f;
  float right_mps = 0.0f;
};

class DifferentialDriveKinematics {
public:
  explicit DifferentialDriveKinematics(float wheel_base_m);

  // Động học thuận (Forward Kinematics):
  // Từ vận tốc 2 bánh xe (m/s) -> Vận tốc tiến v (m/s) và vận tốc góc w (rad/s)
  // Quy ước tọa độ màn hình 2D (trục Y hướng xuống):
  // v_l > v_r -> w > 0 (quay thuận chiều kim đồng hồ / rẽ phải)
  // v_l < v_r -> w < 0 (quay ngược chiều kim đồng hồ / rẽ trái)
  [[nodiscard]] Twist2D forward(float v_left_mps, float v_right_mps) const;

  // Động học nghịch (Inverse Kinematics):
  // Từ vận tốc tiến v (m/s) và vận tốc góc w (rad/s) -> Vận tốc 2 bánh xe (m/s)
  [[nodiscard]] WheelVelocities inverse(float linear_v_mps, float angular_w_radps) const;

  // Chuyển đổi và chuẩn hóa vận tốc bánh xe sang tỷ lệ xung PWM [-1.0, 1.0] (Poka-Yoke)
  static float velocity_to_pwm(float wheel_v_mps, float max_v_mps);

private:
  float wheel_base_m_ = 0.115f;
};

}  // namespace lf
