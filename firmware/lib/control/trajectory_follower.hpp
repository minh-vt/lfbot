#pragma once

#include <cstddef>
#include <vector>

#include "config/robot_config.hpp"
#include "config/types.hpp"
#include "control/pid_controller.hpp"
#include "control/stanley_tracker.hpp"

namespace lf {

class TrajectoryFollower {
public:
  TrajectoryFollower(const PIDConfig& pid_cfg, const ProfilerConfig& prof_cfg,
                     const ObstacleConfig& obs_cfg);

  void reset();
  void set_trajectory(const std::vector<TrajectoryPoint>& trajectory);
  void clear_trajectory() {
    trajectory_.clear();
    last_index_ = 0;
  }
  [[nodiscard]] bool has_trajectory() const { return !trajectory_.empty(); }
  [[nodiscard]] const std::vector<TrajectoryPoint>& get_trajectory() const { return trajectory_; }

  // Execute tracking step:
  // - If global racing trajectory is available (Run 2):
  //     Tracks time-optimal trajectory via continuous Stanley controller
  //     Generates smooth steering angle and differential wheel speeds
  // - If exploring line without trajectory (Run 1):
  //     Tracks line via speed-scaled optical PID with corner deceleration
  void update(const Pose2D& pose, float current_v_mps, float delta_dist_m, float centroid_mm,
              bool line_found, float dt, const SensorData& in, MotorCmd& out, float& out_target_v,
              float& out_target_steer, int& out_waypoint_idx, uint8_t& out_bypass_state);

  [[nodiscard]] PIDController& get_pid() { return line_pid_; }
  [[nodiscard]] const PIDController& get_pid() const { return line_pid_; }
  [[nodiscard]] const StanleyTracker& get_stanley() const { return stanley_; }

  [[nodiscard]] float get_cross_track_error_m() const { return cross_track_error_m_; }
  [[nodiscard]] float get_heading_error_rad() const { return heading_error_rad_; }

  void set_dynamics(float max_straight_speed_mps, float safe_map_speed_mps) {
    prof_cfg_.max_straight_speed_mps = max_straight_speed_mps;
    prof_cfg_.safe_map_speed_mps = safe_map_speed_mps;
  }

  void apply_motor_drive(float v_target, float steer, MotorCmd& out) const;

  PIDController line_pid_;
  ProfilerConfig prof_cfg_;
  ObstacleConfig obs_cfg_;
  StanleyTracker stanley_;
  std::vector<TrajectoryPoint> trajectory_;
  size_t last_index_ = 0;
  bool is_evading_ = false;
  float evasion_origin_x_ = 0.0f;
  float evasion_origin_y_ = 0.0f;
  float evasion_origin_th_ = 0.0f;
  float evasion_side_sign_ = 1.0f;
  float evasion_cooldown_s_ = 0.0f;
  float current_bypass_length_m_ = 0.82f;
  float current_bypass_speed_mps_ = 0.45f;
  float cross_track_error_m_ = 0.0f;
  float heading_error_rad_ = 0.0f;
};

}  // namespace lf
