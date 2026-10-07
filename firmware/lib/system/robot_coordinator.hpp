#pragma once

#include <vector>

#include "config/robot_config.hpp"
#include "config/types.hpp"
#include "control/relay_tuner.hpp"
#include "control/trajectory_follower.hpp"
#include "math/centroid.hpp"
#include "math/odometry.hpp"
#include "track/trajectory_planner.hpp"

namespace lf {

class RobotCoordinator {
public:
  explicit RobotCoordinator(const RobotConfig& config = RobotConfig{});

  void init();
  void reset_pose();
  void set_mode(RobotMode mode);
  [[nodiscard]] RobotMode get_mode() const { return mode_; }

  void update(const SensorData& in, MotorCmd& out);

  [[nodiscard]] const Telemetry& get_telemetry() const { return telemetry_; }
  [[nodiscard]] Telemetry& get_telemetry_mut() { return telemetry_; }
  [[nodiscard]] const RobotConfig& get_config() const { return config_; }

  [[nodiscard]] const TrajectoryPlanner& get_planner() const { return planner_; }
  [[nodiscard]] const TrajectoryFollower& get_follower() const { return follower_; }
  [[nodiscard]] TrajectoryFollower& get_follower_mut() { return follower_; }
  [[nodiscard]] const RelayTuner& get_tuner() const { return tuner_; }
  [[nodiscard]] const OdometryTracker& get_odometry() const { return odometry_; }

  [[nodiscard]] const std::vector<TrajectoryPoint>& get_recorded_trace() const {
    return recorded_trace_;
  }
  [[nodiscard]] const std::vector<TrajectoryPoint>& get_racing_trajectory() const {
    return racing_trajectory_;
  }
  void set_racing_trajectory(const std::vector<TrajectoryPoint>& traj);

  [[nodiscard]] const std::vector<ObstacleGeometry>& get_detected_obstacles() const {
    return detected_obstacles_;
  }
  void add_detected_obstacle(const ObstacleGeometry& obs) { detected_obstacles_.push_back(obs); }

  void report_collision(int obstacle_idx = -1);
  [[nodiscard]] bool is_crashed() const { return telemetry_.collision; }

  // Tactical parameter accessors
  void set_line_pid_gains(float kp, float ki, float kd);
  void get_line_pid_gains(float& kp, float& ki, float& kd) const;
  void set_tactical_speeds(float safe_map_speed, float max_straight_speed);
  void set_profiler_dynamics(float max_accel, float max_brake, float corner_grip);
  void set_obstacle_parameters(float front_detect_dist_m, float clearance_m, float length_m);
  void get_obstacle_parameters(float& front_detect_dist_m, float& clearance_m,
                               float& length_m) const;
  void set_min_finish_arm_dist(float dist_m) { config_.tracking.min_finish_arm_dist_m = dist_m; }

private:
  static void handle_idle_mode(MotorCmd& out);
  void handle_auto_tune_mode(float centroid_mm, bool line_found, float dt, MotorCmd& out);
  void handle_run_1_map_mode(const SensorData& in, float centroid_mm, bool line_found,
                             bool is_finish, float dt, MotorCmd& out);
  void handle_run_2_sprint_mode(const SensorData& in, float centroid_mm, bool line_found,
                                bool is_finish, float dt, MotorCmd& out);
  bool check_finish_condition(bool is_finish, float total_dist, float dt, MotorCmd& out);

  RobotConfig config_;
  RobotMode mode_ = RobotMode::kIdle;
  Telemetry telemetry_{};

  CentroidCalculator centroid_calc_;
  OdometryTracker odometry_;
  RelayTuner tuner_;
  TrajectoryFollower follower_;
  TrajectoryPlanner planner_;

  std::vector<TrajectoryPoint> recorded_trace_;
  std::vector<TrajectoryPoint> racing_trajectory_;
  std::vector<ObstacleGeometry> detected_obstacles_;
  float finish_entry_dist_m_ = -1.0f;
  float tentative_entry_dist_m_ = -1.0f;
  float finish_dwell_time_s_ = 0.0f;
  float last_record_dist_m_ = 0.0f;
};

}  // namespace lf
