#pragma once

#include <vector>

#include "config/robot_config.hpp"
#include "config/types.hpp"

namespace lf {

struct StanleyConfig {
  float k_cross_track = 6.5f;
  float k_soft = 0.25f;
  float k_heading = 1.3f;
  float wheel_base_m = 0.115f;
  float max_steer_rad = 1.50f;
  float lookahead_dist_m = 0.055f;
};

struct TrackingCommand {
  float steering_angle_rad = 0.0f;
  float target_v_mps = 0.0f;
  float left_wheel_mps = 0.0f;
  float right_wheel_mps = 0.0f;
  float cross_track_error_m = 0.0f;
  float heading_error_rad = 0.0f;
  int closest_waypoint_idx = 0;
};

class StanleyTracker {
public:
  StanleyTracker();
  explicit StanleyTracker(const StanleyConfig& config);

  [[nodiscard]] TrackingCommand update(const Pose2D& pose, float current_v_mps,
                                       const std::vector<TrajectoryPoint>& trajectory,
                                       size_t hint_idx = 0) const;

  [[nodiscard]] const StanleyConfig& get_config() const { return config_; }

private:
  StanleyConfig config_;

  [[nodiscard]] static float normalize_angle(float angle_rad);
};

}  // namespace lf
