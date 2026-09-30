#pragma once

#include "config/robot_config.hpp"
#include "config/types.hpp"
#include "control/pid_controller.hpp"
#include "control/relay_tuner.hpp"
#include "control/velocity_profiler.hpp"
#include "fsm/obstacle_fsm.hpp"
#include "math/centroid.hpp"
#include "math/odometry.hpp"

namespace lf {

class RobotCoordinator {
public:
  explicit RobotCoordinator(const RobotConfig& config = RobotConfig{});

  void init();
  void reset_pose();
  void set_mode(RobotMode mode);
  [[nodiscard]] RobotMode get_mode() const { return mode_; }

  void update(const SensorData& in, MotorCmd& out);
  void start_auto_tune();

  [[nodiscard]] const Telemetry& get_telemetry() const { return telemetry_; }
  [[nodiscard]] Telemetry& get_telemetry_mut() { return telemetry_; }
  [[nodiscard]] const VelocityProfiler& get_profiler() const { return profiler_; }
  [[nodiscard]] const RobotConfig& get_config() const { return config_; }

  void set_line_pid_gains(float kp, float ki, float kd);
  void get_line_pid_gains(float& kp, float& ki, float& kd) const;
  void set_tactical_speeds(float safe_map_speed, float max_straight_speed);
  void set_profiler_dynamics(float max_accel, float max_brake, float corner_grip);
  void set_obstacle_distances(float front_detect_dist_m, float approach_stop_dist_m);
  void get_obstacle_distances(float& front_detect_dist_m, float& approach_stop_dist_m) const;
  void set_obstacle_parameters(float front_detect_dist_m, float clearance_m, float length_m);
  void get_obstacle_parameters(float& front_detect_dist_m, float& clearance_m,
                               float& length_m) const;

private:
  static void handle_idle_mode(MotorCmd& out);
  void handle_auto_tune_mode(float centroid_mm, float dt, MotorCmd& out);
  void handle_run_1_map_mode(const SensorData& in, float centroid_mm, bool line_found,
                             bool is_finish, float dt, MotorCmd& out);
  void handle_run_2_sprint_mode(const SensorData& in, float centroid_mm, bool line_found,
                                bool is_finish, float dt, MotorCmd& out);
  bool check_finish_condition(bool is_finish, float total_dist, MotorCmd& out);
  void apply_motor_drive(float v_target, float steer, MotorCmd& out) const;

  RobotConfig config_;
  RobotMode mode_ = RobotMode::kIdle;
  Telemetry telemetry_{};

  CentroidCalculator centroid_calc_;
  OdometryTracker odometry_;
  PIDController line_pid_;
  PIDController speed_pid_l_;
  PIDController speed_pid_r_;
  RelayAutoTuner auto_tuner_;
  ObstacleFSM obstacle_fsm_;
  VelocityProfiler profiler_;

  float segment_start_dist_m_ = 0.0f;
  SegmentType current_segment_type_ = SegmentType::kStraight;
  float finish_entry_dist_m_ = -1.0f;
};

}  // namespace lf
