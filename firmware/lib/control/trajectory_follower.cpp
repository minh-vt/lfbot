#include "control/trajectory_follower.hpp"

#include <algorithm>
#include <cmath>

namespace lf {

TrajectoryFollower::TrajectoryFollower(const PIDConfig& pid_cfg, const ProfilerConfig& prof_cfg)
  : line_pid_(pid_cfg), prof_cfg_(prof_cfg), stanley_(StanleyConfig{}) {}

void TrajectoryFollower::reset() {
  line_pid_.reset();
  last_index_ = 0;
  cross_track_error_m_ = 0.0f;
  heading_error_rad_ = 0.0f;
}

void TrajectoryFollower::set_trajectory(const std::vector<TrajectoryPoint>& trajectory) {
  trajectory_ = trajectory;
  last_index_ = 0;
}

KinematicCmd TrajectoryFollower::track_trajectory(const Pose2D& pose, float current_v_mps,
                                                  float /*dt*/) {
  KinematicCmd cmd{};
  if (trajectory_.empty()) {
    return cmd;
  }

  const auto stanley_cmd = stanley_.update(pose, current_v_mps, trajectory_, last_index_);
  cmd.closest_waypoint_idx = stanley_cmd.closest_waypoint_idx;
  last_index_ = static_cast<size_t>(std::max(0, stanley_cmd.closest_waypoint_idx));
  cmd.cross_track_error_m = stanley_cmd.cross_track_error_m;
  cmd.heading_error_rad = stanley_cmd.heading_error_rad;
  cmd.target_v_mps = stanley_cmd.target_v_mps;
  cmd.steering_angle_rad = stanley_cmd.steering_angle_rad;
  cmd.left_wheel_mps = stanley_cmd.left_wheel_mps;
  cmd.right_wheel_mps = stanley_cmd.right_wheel_mps;

  cross_track_error_m_ = cmd.cross_track_error_m;
  heading_error_rad_ = cmd.heading_error_rad;
  return cmd;
}

KinematicCmd TrajectoryFollower::track_line(float lateral_error_m, float current_v_mps,
                                            float target_v_mps, float dt) {
  KinematicCmd cmd{};
  cmd.target_v_mps = target_v_mps;

  // Convert lateral error in meters to millimeters for PID controller
  const float error_mm = lateral_error_m * 1000.0f;
  const float pid_steer = line_pid_.update_speed_scaled(error_mm, dt, current_v_mps);

  // Speed adaptation on tight curves
  const float abs_steer = std::abs(pid_steer);
  if (abs_steer > 0.35f) {
    const float decel_factor = std::clamp(1.0f - 0.70f * (abs_steer - 0.35f), 0.35f, 1.0f);
    cmd.target_v_mps *= decel_factor;
  }

  cmd.steering_angle_rad = pid_steer;
  cmd.cross_track_error_m = lateral_error_m;
  cmd.heading_error_rad = 0.0f;
  cmd.closest_waypoint_idx = 0;

  cmd.left_wheel_mps = cmd.target_v_mps + cmd.steering_angle_rad;
  cmd.right_wheel_mps = cmd.target_v_mps - cmd.steering_angle_rad;

  cross_track_error_m_ = cmd.cross_track_error_m;
  heading_error_rad_ = cmd.heading_error_rad;
  return cmd;
}

}  // namespace lf
