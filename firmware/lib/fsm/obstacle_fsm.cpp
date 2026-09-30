#include "fsm/obstacle_fsm.hpp"

#include <algorithm>
#include <cmath>

namespace lf {

ObstacleFSM::ObstacleFSM() : config_(ObstacleConfig{}) {
  reset();
}

ObstacleFSM::ObstacleFSM(const ObstacleConfig& config) : config_(config) {
  reset();
}

void ObstacleFSM::reset() {
  state_ = State::kTracking;
  timer_s_ = 0.0f;
  bypass_direction_ = 1.0f;
  progress_dist_m_ = 0.0f;
}

void ObstacleFSM::update(const SensorData& in, float current_speed, MotorCmd& cmd,
                         bool any_ir_on_line) {
  if (state_ == State::kTracking) {
    if (in.tof_front_m > config_.min_valid_tof_m && in.tof_front_m < config_.front_detect_dist_m) {
      state_ = State::kArcChicane;
      timer_s_ = 0.0f;
      progress_dist_m_ = 0.0f;
      bypass_direction_ = (in.tof_left_m > in.tof_right_m + 0.02f) ? -1.0f : 1.0f;
    } else {
      return;
    }
  }

  timer_s_ += in.dt;
  if (timer_s_ > config_.emergency_timeout_s && state_ != State::kEmergencySearch) {
    state_ = State::kEmergencySearch;
  }

  const float max_speed =
    (config_.max_straight_speed_mps > 0.1f) ? config_.max_straight_speed_mps : 2.50f;

  if (state_ == State::kArcChicane) {
    const float l_chicane = std::max(0.20f, config_.bypass_length_m);

    // Bat lai vach khi da vuot qua nua quang duong va mat quang cham vach
    if (progress_dist_m_ >= l_chicane * 0.45f && any_ir_on_line) {
      state_ = State::kTracking;
      timer_s_ = 0.0f;
      progress_dist_m_ = 0.0f;
      return;
    }

    progress_dist_m_ += std::max(0.10f, current_speed) * in.dt;

    // Do cong hinh sin: kappa(s) = 0.5 * Y * (2*pi/L)^2 * cos(2*pi*s/L)
    constexpr float kTwoPi = 6.2831853f;
    const float omega_s = kTwoPi / l_chicane;
    const float kappa =
      0.5f * config_.bypass_clearance_m * omega_s * omega_s * std::cos(omega_s * progress_dist_m_);

    const float v_base = config_.bypass_speed_mps;
    const float delta_v = bypass_direction_ * v_base * kappa * (config_.wheel_base_m * 0.5f);

    cmd.left_pwm = std::clamp((v_base + delta_v) / max_speed, -0.6f, 0.8f);
    cmd.right_pwm = std::clamp((v_base - delta_v) / max_speed, -0.6f, 0.8f);
    cmd.active_brake = false;

    if (progress_dist_m_ > l_chicane * 1.3f) {
      state_ = State::kEmergencySearch;
    }
    return;
  }

  if (state_ == State::kEmergencySearch) {
    cmd.left_pwm = -config_.emergency_spin_pwm;
    cmd.right_pwm = config_.emergency_spin_pwm;
    cmd.active_brake = false;

    if (any_ir_on_line) {
      state_ = State::kTracking;
      timer_s_ = 0.0f;
      progress_dist_m_ = 0.0f;
    }
  }
}

}  // namespace lf
