#include "math/kinematics.hpp"

#include <algorithm>

namespace lf {

DifferentialDriveKinematics::DifferentialDriveKinematics(float wheel_base_m)
  : wheel_base_m_((wheel_base_m > 0.001f) ? wheel_base_m : 0.115f) {}

Twist2D DifferentialDriveKinematics::forward(float v_left_mps, float v_right_mps) const {
  Twist2D twist{};
  twist.linear_v_mps = 0.5f * (v_left_mps + v_right_mps);
  twist.angular_w_radps = (v_left_mps - v_right_mps) / wheel_base_m_;
  return twist;
}

float DifferentialDriveKinematics::velocity_to_pwm(float wheel_v_mps, float max_v_mps) {
  // Guard clause: Chống chia cho 0 nếu max_v không hợp lệ (Poka-Yoke)
  if (max_v_mps <= 0.0001f) {
    return 0.0f;
  }

  return std::clamp(wheel_v_mps / max_v_mps, -1.0f, 1.0f);
}

}  // namespace lf
