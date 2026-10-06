#pragma once

#include <cstdint>

#include "config/robot_config.hpp"
#include "config/types.hpp"

namespace lf {

class OdometryTracker {
public:
  explicit OdometryTracker(const KinematicsConfig& config);

  void reset();
  void update(int32_t enc_left, int32_t enc_right, float dt);

  [[nodiscard]] float get_total_distance_m() const { return total_distance_m_; }
  [[nodiscard]] float get_current_velocity_mps() const { return current_velocity_mps_; }
  [[nodiscard]] float get_delta_distance_m() const { return last_delta_dist_m_; }
  [[nodiscard]] float get_left_velocity_mps() const { return vl_mps_; }
  [[nodiscard]] float get_right_velocity_mps() const { return vr_mps_; }
  [[nodiscard]] float get_heading_rad() const { return heading_rad_; }
  [[nodiscard]] float get_x_m() const { return x_m_; }
  [[nodiscard]] float get_y_m() const { return y_m_; }
  [[nodiscard]] float get_angular_velocity_radps() const { return angular_velocity_radps_; }
  [[nodiscard]] Pose2D get_pose() const { return Pose2D{x_m_, y_m_, heading_rad_}; }

private:
  KinematicsConfig config_;
  int32_t prev_enc_l_ = 0;
  int32_t prev_enc_r_ = 0;
  bool is_initialized_ = false;

  float total_distance_m_ = 0.0f;
  float current_velocity_mps_ = 0.0f;
  float last_delta_dist_m_ = 0.0f;
  float vl_mps_ = 0.0f;
  float vr_mps_ = 0.0f;
  float heading_rad_ = 0.0f;
  float x_m_ = 0.0f;
  float y_m_ = 0.0f;
  float angular_velocity_radps_ = 0.0f;
};

}  // namespace lf
