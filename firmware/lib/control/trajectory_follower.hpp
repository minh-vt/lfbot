#pragma once

#include <cstddef>
#include <vector>

#include "config/robot_config.hpp"
#include "config/types.hpp"
#include "control/pid_controller.hpp"
#include "control/stanley_tracker.hpp"

namespace lf {

struct KinematicCmd {
  float target_v_mps = 0.0f;
  float steering_angle_rad = 0.0f;
  float omega_rad_s = 0.0f;
  float left_wheel_mps = 0.0f;
  float right_wheel_mps = 0.0f;
  int closest_waypoint_idx = 0;
  float cross_track_error_m = 0.0f;
  float heading_error_rad = 0.0f;
};

class TrajectoryFollower {
public:
  TrajectoryFollower(const PIDConfig& pid_cfg, const ProfilerConfig& prof_cfg);

  void reset();
  void set_trajectory(const std::vector<TrajectoryPoint>& trajectory);
  void clear_trajectory() {
    trajectory_.clear();
    last_index_ = 0;
  }
  [[nodiscard]] bool has_trajectory() const { return !trajectory_.empty(); }
  [[nodiscard]] const std::vector<TrajectoryPoint>& get_trajectory() const { return trajectory_; }

  // 1. Run 2 Sprint: Stanley tracking controller on global C^2 racing line
  KinematicCmd track_trajectory(const Pose2D& pose, float current_v_mps, float dt);

  // 2. Run 1 Map: Speed-scaled PID lateral tracking controller on optical / tactical error
  KinematicCmd track_line(float lateral_error_m, float current_v_mps, float target_v_mps, float dt);

  [[nodiscard]] PIDController& get_pid() { return line_pid_; }
  [[nodiscard]] const PIDController& get_pid() const { return line_pid_; }
  [[nodiscard]] const StanleyTracker& get_stanley() const { return stanley_; }

  [[nodiscard]] float get_cross_track_error_m() const { return cross_track_error_m_; }
  [[nodiscard]] float get_heading_error_rad() const { return heading_error_rad_; }

  void set_dynamics(float max_straight_speed_mps, float safe_map_speed_mps) {
    prof_cfg_.max_straight_speed_mps = max_straight_speed_mps;
    prof_cfg_.safe_map_speed_mps = safe_map_speed_mps;
  }

private:
  PIDController line_pid_;
  ProfilerConfig prof_cfg_;
  StanleyTracker stanley_;
  std::vector<TrajectoryPoint> trajectory_;
  size_t last_index_ = 0;
  float cross_track_error_m_ = 0.0f;
  float heading_error_rad_ = 0.0f;
};

}  // namespace lf
