#pragma once

#include <cstdint>

#include "config/robot_config.hpp"

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
};

}  // namespace lf
