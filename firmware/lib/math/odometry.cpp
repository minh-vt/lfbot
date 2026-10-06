#include "math/odometry.hpp"

#include <cmath>

namespace lf {

OdometryTracker::OdometryTracker(const KinematicsConfig& config) : config_(config) {
  reset();
}

void OdometryTracker::reset() {
  prev_enc_l_ = 0;
  prev_enc_r_ = 0;
  is_initialized_ = false;
  total_distance_m_ = 0.0f;
  current_velocity_mps_ = 0.0f;
  last_delta_dist_m_ = 0.0f;
  vl_mps_ = 0.0f;
  vr_mps_ = 0.0f;
  heading_rad_ = 0.0f;
  x_m_ = 0.0f;
  y_m_ = 0.0f;
  angular_velocity_radps_ = 0.0f;
}

void OdometryTracker::update(int32_t enc_left, int32_t enc_right, float dt) {
  // Guard clause: Lần đầu tiên ghi nhận giá trị gốc
  if (!is_initialized_) {
    prev_enc_l_ = enc_left;
    prev_enc_r_ = enc_right;
    is_initialized_ = true;
    return;
  }

  // Guard clause: dt không hợp lệ
  if (dt <= 0.00001f) {
    return;
  }

  const int32_t delta_l = enc_left - prev_enc_l_;
  const int32_t delta_r = enc_right - prev_enc_r_;
  prev_enc_l_ = enc_left;
  prev_enc_r_ = enc_right;

  const float m_per_tick = config_.meters_per_tick();
  const float dist_l = static_cast<float>(delta_l) * m_per_tick;
  const float dist_r = static_cast<float>(delta_r) * m_per_tick;

  last_delta_dist_m_ = 0.5f * (dist_l + dist_r);
  total_distance_m_ += last_delta_dist_m_;
  current_velocity_mps_ = last_delta_dist_m_ / dt;
  vl_mps_ = dist_l / dt;
  vr_mps_ = dist_r / dt;

  if (config_.wheel_base_m > 0.001f) {
    const float delta_theta = (dist_l - dist_r) / config_.wheel_base_m;
    const float mid_heading = heading_rad_ + 0.5f * delta_theta;
    heading_rad_ += delta_theta;
    angular_velocity_radps_ = (vl_mps_ - vr_mps_) / config_.wheel_base_m;
    x_m_ += last_delta_dist_m_ * std::cos(mid_heading);
    y_m_ += last_delta_dist_m_ * std::sin(mid_heading);
  } else {
    x_m_ += last_delta_dist_m_ * std::cos(heading_rad_);
    y_m_ += last_delta_dist_m_ * std::sin(heading_rad_);
  }
}

}  // namespace lf
