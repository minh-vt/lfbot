#include "control/pid_controller.hpp"

#include <algorithm>
#include <cmath>

namespace lf {

PIDController::PIDController() = default;

PIDController::PIDController(const PIDConfig& config) : config_(config) {
  reset();
}

void PIDController::set_config(const PIDConfig& config) {
  config_ = config;
}

void PIDController::set_gains(float kp, float ki, float kd) {
  config_.kp = kp;
  config_.ki = ki;
  config_.kd = kd;
}

void PIDController::reset() {
  integral_ = 0.0f;
  prev_error_ = 0.0f;
  prev_d_term_ = 0.0f;
}

float PIDController::update(float error, float dt) {
  // Guard clause: Khoảng thời gian chu kỳ không hợp lệ
  if (dt <= 0.00001f) {
    return 0.0f;
  }

  // Khâu Tỷ lệ (P)
  const float p_term = config_.kp * error;

  // Khâu Tích phân (I)
  integral_ += error * dt;
  const float i_term = config_.ki * integral_;

  // Khâu Vi phân (D) có bộ lọc thông thấp (LPF)
  const float raw_derivative = (error - prev_error_) / dt;
  const float alpha = config_.derivative_lpf_alpha;
  const float filtered_derivative = alpha * prev_d_term_ + (1.0f - alpha) * raw_derivative;
  prev_d_term_ = filtered_derivative;
  const float d_term = config_.kd * filtered_derivative;

  prev_error_ = error;

  const float output = p_term + i_term + d_term;

  // Guard clause / Anti-windup clamping
  if (output > config_.out_max) {
    if (error > 0.0f) {
      integral_ -= error * dt;
    }
    return config_.out_max;
  }

  if (output < config_.out_min) {
    if (error < 0.0f) {
      integral_ -= error * dt;
    }
    return config_.out_min;
  }

  return output;
}

float PIDController::update_speed_scaled(float error, float dt, float current_speed,
                                         float ref_speed) {
  const float v_ref = (ref_speed > 0.01f) ? ref_speed : config_.speed_scale_v_ref;
  const float scale = std::clamp(v_ref / std::max(0.10f, current_speed), config_.speed_scale_min,
                                 config_.speed_scale_max);
  return std::clamp(update(error, dt) * scale, config_.out_min, config_.out_max);
}

}  // namespace lf
