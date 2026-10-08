#include "system/robot_coordinator.hpp"

#include <algorithm>
#include <cmath>

#include "math/kinematics.hpp"

namespace lf {

RobotCoordinator::RobotCoordinator(const RobotConfig& config)
  : config_(config),
    centroid_calc_(config.line_sensor),
    odometry_(config.kinematics),
    tuner_(0.35f, 3.5f),
    arbiter_(config.obstacle, config.profiler, config.tracking),
    follower_(config.line_pid, config.profiler),
    planner_(config.environment, config.profiler) {
  init();
}

void RobotCoordinator::init() {
  mode_ = RobotMode::kIdle;
  telemetry_ = {};
  finish_entry_dist_m_ = -1.0f;
  last_record_dist_m_ = 0.0f;

  odometry_.reset();
  arbiter_.reset();
  follower_.reset();
  recorded_trace_.clear();
  racing_trajectory_.clear();
  detected_obstacles_.clear();
}

void RobotCoordinator::reset_pose() {
  mode_ = RobotMode::kIdle;
  telemetry_ = {};
  finish_entry_dist_m_ = -1.0f;
  tentative_entry_dist_m_ = -1.0f;
  finish_dwell_time_s_ = 0.0f;
  last_record_dist_m_ = 0.0f;

  odometry_.reset();
  arbiter_.reset();
  follower_.reset();
  // Preserve recorded_trace_, racing_trajectory_, and detected_obstacles_ for Run 2
}

void RobotCoordinator::set_mode(RobotMode mode) {
  mode_ = mode;
  follower_.reset();
  finish_entry_dist_m_ = -1.0f;
  tentative_entry_dist_m_ = -1.0f;
  finish_dwell_time_s_ = 0.0f;
  telemetry_.collision = false;
  telemetry_.obstacle.collision = false;

  if (mode == RobotMode::kAutoTune) {
    odometry_.reset();
    tuner_.reset();
    return;
  }

  if (mode == RobotMode::kRun1Map) {
    odometry_.reset();
    recorded_trace_.clear();
    racing_trajectory_.clear();
    detected_obstacles_.clear();
    last_record_dist_m_ = 0.0f;
    return;
  }

  if (mode == RobotMode::kRun2Sprint) {
    odometry_.reset();
    if (!racing_trajectory_.empty()) {
      follower_.set_trajectory(racing_trajectory_);
    }
    return;
  }
}

void RobotCoordinator::report_collision(int obstacle_idx) {
  mode_ = RobotMode::kIdle;
  telemetry_.collision = true;
  telemetry_.obstacle.collision = true;
  (void)obstacle_idx;
}

void RobotCoordinator::set_racing_trajectory(const std::vector<TrajectoryPoint>& traj) {
  racing_trajectory_ = traj;
  follower_.set_trajectory(traj);
}

bool RobotCoordinator::reoptimize_racing_trajectory() {
  if (recorded_trace_.size() < 10) {
    return false;
  }
  racing_trajectory_ = planner_.generate_racing_line(recorded_trace_, detected_obstacles_);
  follower_.set_trajectory(racing_trajectory_);
  return true;
}

void RobotCoordinator::handle_idle_mode(MotorCmd& out) {
  out.left_pwm = 0.0f;
  out.right_pwm = 0.0f;
  out.active_brake = true;
}

bool RobotCoordinator::poll_finish_line_entry(bool is_finish, float total_dist, float dt) {
  if (mode_ == RobotMode::kRun2Sprint && follower_.has_trajectory() &&
      !racing_trajectory_.empty()) {
    if (!is_finish) {
      finish_dwell_time_s_ = 0.0f;
      tentative_entry_dist_m_ = -1.0f;
      return false;
    }
    finish_entry_dist_m_ = total_dist;
    return true;
  }

  if (total_dist <= config_.tracking.min_finish_arm_dist_m) {
    finish_dwell_time_s_ = 0.0f;
    tentative_entry_dist_m_ = -1.0f;
    return false;
  }
  if (!is_finish) {
    finish_dwell_time_s_ = 0.0f;
    tentative_entry_dist_m_ = -1.0f;
    return false;
  }

  if (tentative_entry_dist_m_ < 0.0f) {
    tentative_entry_dist_m_ = total_dist;
  }
  finish_dwell_time_s_ += dt;
  if (finish_dwell_time_s_ < config_.tracking.finish_debounce_time_s) {
    return false;
  }
  finish_entry_dist_m_ = tentative_entry_dist_m_;
  return true;
}

bool RobotCoordinator::check_finish_condition(bool is_finish, float total_dist, float dt,
                                              MotorCmd& out) {
  if (finish_entry_dist_m_ < 0.0f) {
    if (!poll_finish_line_entry(is_finish, total_dist, dt)) {
      return false;
    }

    if (mode_ == RobotMode::kRun1Map && recorded_trace_.size() > 10) {
      // Auto-generate global QP time-optimal racing line incorporating detected obstacles
      racing_trajectory_ = planner_.generate_racing_line(recorded_trace_, detected_obstacles_);
      follower_.set_trajectory(racing_trajectory_);
    }
  }

  const float dist_in_finish = total_dist - finish_entry_dist_m_;
  if (dist_in_finish < config_.tracking.finish_zone_length_m) {
    const float finish_speed =
      config_.profiler.safe_map_speed_mps * config_.tracking.finish_speed_factor;
    apply_motor_drive(finish_speed, 0.0f, out);
    return true;
  }

  set_mode(RobotMode::kIdle);
  out.left_pwm = 0.0f;
  out.right_pwm = 0.0f;
  out.active_brake = true;
  return true;
}

void RobotCoordinator::handle_auto_tune_mode(float centroid_mm, bool line_found, float dt,
                                             MotorCmd& out) {
  if (!line_found) {
    handle_idle_mode(out);
    return;
  }

  const float tuner_speed = std::clamp(config_.profiler.safe_map_speed_mps * 0.5f, 0.20f, 0.40f);
  const float steer = tuner_.update(centroid_mm, dt);
  apply_motor_drive(tuner_speed, steer, out);

  telemetry_.tuner_cycles = tuner_.get_cycle_count();
  telemetry_.tuner_amplitude_mm = tuner_.get_current_amplitude_mm();

  if (tuner_.is_converged()) {
    const auto res = tuner_.get_result();
    set_line_pid_gains(res.kp, 0.0f, res.kd);
    set_mode(RobotMode::kIdle);
    handle_idle_mode(out);
  }
}

void RobotCoordinator::handle_run_1_map_mode(const SensorData& in, float centroid_mm,
                                             bool line_found, bool is_finish, float dt,
                                             MotorCmd& out) {
  const float total_dist = odometry_.get_total_distance_m();

  if (check_finish_condition(is_finish, total_dist, dt, out)) {
    return;
  }

  // Record spatial waypoints every 15mm of travel
  constexpr float kSampleSpatialStepM = 0.015f;
  if (recorded_trace_.empty() || (total_dist - last_record_dist_m_) >= kSampleSpatialStepM) {
    last_record_dist_m_ = total_dist;
    TrajectoryPoint pt{};
    pt.x_mm = odometry_.get_x_m() * 1000.0f;
    pt.y_mm = odometry_.get_y_m() * 1000.0f;
    pt.s_m = total_dist;
    pt.heading_rad = odometry_.get_heading_rad();
    pt.target_speed_mps = config_.profiler.safe_map_speed_mps;
    pt.curvature_rad_m = 0.0f;
    recorded_trace_.push_back(pt);
  }

  // Register detected obstacles into map
  if (in.tof_front_m > config_.obstacle.min_valid_tof_m &&
      in.tof_front_m < config_.obstacle.front_detect_dist_m) {
    const float obs_dist_m = in.tof_front_m + 0.5f * config_.kinematics.wheel_base_m;
    const float obs_x_mm =
      (odometry_.get_x_m() + obs_dist_m * std::cos(odometry_.get_heading_rad())) * 1000.0f;
    const float obs_y_mm =
      (odometry_.get_y_m() + obs_dist_m * std::sin(odometry_.get_heading_rad())) * 1000.0f;

    bool exists = false;
    for (const auto& obs : detected_obstacles_) {
      const float dx = obs.x_mm - obs_x_mm;
      const float dy = obs.y_mm - obs_y_mm;
      if (std::sqrt(dx * dx + dy * dy) < config_.environment.obstacle_merge_dist_mm) {
        exists = true;
        break;
      }
    }
    if (!exists) {
      detected_obstacles_.push_back({
        obs_x_mm,
        obs_y_mm,
        config_.environment.obstacle_length_mm,
        config_.environment.obstacle_width_mm,
        0.0f,
      });
    }
  }

  // 1. Tactical Behavior Arbiter evaluates perception, APF, and line loss
  const auto tactical_ref =
    arbiter_.update(odometry_.get_pose(), odometry_.get_current_velocity_mps(), centroid_mm,
                    line_found, in.tof_front_m, in.tof_left_m, in.tof_right_m, dt, mode_);

  telemetry_.bypass_state = tactical_ref.bypass_state;
  telemetry_.current_segment_idx = 0;

  if (tactical_ref.should_halt) {
    apply_motor_drive(0.0f, 0.0f, out);
    telemetry_.target_velocity_mps = 0.0f;
    telemetry_.stanley_steer_rad = 0.0f;
    return;
  }

  if (tactical_ref.has_steer_override) {
    apply_motor_drive(tactical_ref.target_v_mps, tactical_ref.steer_cmd_rad, out);
    telemetry_.target_velocity_mps = tactical_ref.target_v_mps;
    telemetry_.stanley_steer_rad = tactical_ref.steer_cmd_rad;
    telemetry_.cross_track_error_m = 0.0f;
    telemetry_.heading_error_rad = 0.0f;
    return;
  }

  // 2. Trajectory Follower computes kinematic command
  const auto kin_cmd =
    follower_.track_line(tactical_ref.lateral_error_m, odometry_.get_current_velocity_mps(),
                         tactical_ref.target_v_mps, dt);

  // 3. Actuation mapping to differential drive PWM
  apply_motor_drive(kin_cmd.target_v_mps, kin_cmd.steering_angle_rad, out);
  telemetry_.target_velocity_mps = kin_cmd.target_v_mps;
  telemetry_.stanley_steer_rad = kin_cmd.steering_angle_rad;
  telemetry_.cross_track_error_m = kin_cmd.cross_track_error_m;
  telemetry_.heading_error_rad = kin_cmd.heading_error_rad;
}

void RobotCoordinator::handle_run_2_sprint_mode(const SensorData& in, float centroid_mm,
                                                bool line_found, bool is_finish, float dt,
                                                MotorCmd& out) {
  const float total_dist = odometry_.get_total_distance_m();

  if (check_finish_condition(is_finish, total_dist, dt, out)) {
    return;
  }

  // Tactical Behavior Arbiter
  const auto tactical_ref =
    arbiter_.update(odometry_.get_pose(), odometry_.get_current_velocity_mps(), centroid_mm,
                    line_found, in.tof_front_m, in.tof_left_m, in.tof_right_m, dt, mode_);

  telemetry_.bypass_state = tactical_ref.bypass_state;

  if (tactical_ref.should_halt) {
    apply_motor_drive(0.0f, 0.0f, out);
    telemetry_.target_velocity_mps = 0.0f;
    telemetry_.stanley_steer_rad = 0.0f;
    return;
  }

  if (tactical_ref.has_steer_override) {
    apply_motor_drive(tactical_ref.target_v_mps, tactical_ref.steer_cmd_rad, out);
    telemetry_.target_velocity_mps = tactical_ref.target_v_mps;
    telemetry_.stanley_steer_rad = tactical_ref.steer_cmd_rad;
    return;
  }

  KinematicCmd kin_cmd{};
  if (follower_.has_trajectory()) {
    kin_cmd =
      follower_.track_trajectory(odometry_.get_pose(), odometry_.get_current_velocity_mps(), dt);
  } else {
    kin_cmd =
      follower_.track_line(tactical_ref.lateral_error_m, odometry_.get_current_velocity_mps(),
                           tactical_ref.target_v_mps, dt);
  }

  apply_motor_drive(kin_cmd.target_v_mps, kin_cmd.steering_angle_rad, out);
  telemetry_.current_segment_idx = kin_cmd.closest_waypoint_idx;
  telemetry_.cross_track_error_m = kin_cmd.cross_track_error_m;
  telemetry_.heading_error_rad = kin_cmd.heading_error_rad;
  telemetry_.target_velocity_mps = kin_cmd.target_v_mps;
  telemetry_.stanley_steer_rad = kin_cmd.steering_angle_rad;
}

void RobotCoordinator::update(const SensorData& in, MotorCmd& out) {
  float centroid_mm = telemetry_.centroid_pos_mm;
  const bool line_found = centroid_calc_.compute(in.ir_raw, centroid_mm);
  const float total_dist_so_far = odometry_.get_total_distance_m();
  bool is_finish = false;
  if (mode_ == RobotMode::kRun2Sprint && follower_.has_trajectory() &&
      !racing_trajectory_.empty()) {
    const float end_s = racing_trajectory_.back().s_m;
    is_finish = (total_dist_so_far >= end_s - 0.20f);
  } else {
    is_finish = (total_dist_so_far > config_.tracking.min_finish_arm_dist_m) && (!line_found);
  }

  if (line_found) {
    telemetry_.centroid_pos_mm = centroid_mm;
  } else {
    centroid_mm = 0.0f;
    telemetry_.centroid_pos_mm = 0.0f;
  }

  odometry_.update(in.enc_left_ticks, in.enc_right_ticks, in.dt);

  telemetry_.line_lost = !line_found;
  telemetry_.total_distance_m = odometry_.get_total_distance_m();
  telemetry_.current_velocity_mps = odometry_.get_current_velocity_mps();
  telemetry_.angular_velocity_radps = odometry_.get_angular_velocity_radps();
  telemetry_.heading_rad = odometry_.get_heading_rad();
  telemetry_.mode = mode_;

  telemetry_.wheel_l.actual_speed_mps = odometry_.get_left_velocity_mps();
  telemetry_.wheel_l.enc_ticks = in.enc_left_ticks;
  telemetry_.wheel_r.actual_speed_mps = odometry_.get_right_velocity_mps();
  telemetry_.wheel_r.enc_ticks = in.enc_right_ticks;

  switch (mode_) {
    case RobotMode::kIdle:
      handle_idle_mode(out);
      break;
    case RobotMode::kAutoTune:
      handle_auto_tune_mode(centroid_mm, line_found, in.dt, out);
      break;
    case RobotMode::kRun1Map:
      handle_run_1_map_mode(in, centroid_mm, line_found, is_finish, in.dt, out);
      break;
    case RobotMode::kRun2Sprint:
      handle_run_2_sprint_mode(in, centroid_mm, line_found, is_finish, in.dt, out);
      break;
    default:
      handle_idle_mode(out);
      break;
  }

  telemetry_.wheel_l.pwm_duty = out.left_pwm;
  telemetry_.wheel_r.pwm_duty = out.right_pwm;

  telemetry_.obstacle.tof_front_m = in.tof_front_m;
  telemetry_.obstacle.tof_left_m = in.tof_left_m;
  telemetry_.obstacle.tof_right_m = in.tof_right_m;
  telemetry_.obstacle.nearest_dist_m = std::min({in.tof_front_m, in.tof_left_m, in.tof_right_m});
  telemetry_.obstacle.detected = (in.tof_front_m < config_.obstacle.front_detect_dist_m);
}

void RobotCoordinator::set_line_pid_gains(float kp, float ki, float kd) {
  config_.line_pid.kp = std::clamp(kp, 0.005f, 0.200f);
  config_.line_pid.ki = std::clamp(ki, 0.0f, 0.050f);
  config_.line_pid.kd = std::clamp(kd, 0.0005f, 0.050f);
  follower_.get_pid().set_gains(config_.line_pid.kp, config_.line_pid.ki, config_.line_pid.kd);
}

void RobotCoordinator::get_line_pid_gains(float& kp, float& ki, float& kd) const {
  follower_.get_pid().get_gains(kp, ki, kd);
}

void RobotCoordinator::set_tactical_speeds(float safe_map_speed, float max_straight_speed) {
  config_.profiler.safe_map_speed_mps = std::clamp(safe_map_speed, 0.20f, 1.50f);
  config_.profiler.max_straight_speed_mps = std::clamp(max_straight_speed, 0.50f, 4.00f);
  follower_.set_dynamics(config_.profiler.max_straight_speed_mps,
                         config_.profiler.safe_map_speed_mps);
  arbiter_.set_profiler_config(config_.profiler);
  planner_ =
    TrajectoryPlanner(config_.environment, config_.profiler, planner_.get_optimizer_config());
}

void RobotCoordinator::set_profiler_dynamics(float max_accel, float max_brake, float corner_grip) {
  config_.profiler.max_accel_mps2 = std::clamp(max_accel, 1.0f, 6.0f);
  config_.profiler.max_brake_mps2 = std::clamp(max_brake, 1.5f, 8.0f);
  config_.profiler.corner_grip_mu = std::clamp(corner_grip, 0.3f, 1.2f);
  planner_ =
    TrajectoryPlanner(config_.environment, config_.profiler, planner_.get_optimizer_config());
}

void RobotCoordinator::set_obstacle_parameters(float front_detect_dist_m, float clearance_m,
                                               float length_m) {
  config_.obstacle.front_detect_dist_m = std::clamp(front_detect_dist_m, 0.08f, 0.60f);
  config_.obstacle.bypass_clearance_m = std::clamp(clearance_m, 0.08f, 0.35f);
  config_.obstacle.bypass_length_m = std::clamp(length_m, 0.30f, 1.50f);
  arbiter_.set_obstacle_config(config_.obstacle);
}

void RobotCoordinator::get_obstacle_parameters(float& front_detect_dist_m, float& clearance_m,
                                               float& length_m) const {
  front_detect_dist_m = config_.obstacle.front_detect_dist_m;
  clearance_m = config_.obstacle.bypass_clearance_m;
  length_m = config_.obstacle.bypass_length_m;
}

void RobotCoordinator::set_optimizer_params(float max_lat_dev_mm, float jerk_weight) {
  auto opt_cfg = planner_.get_optimizer_config();
  opt_cfg.max_lateral_deviation_mm = std::clamp(max_lat_dev_mm, 0.0f, 120.0f);
  opt_cfg.jerk_damping_weight = std::clamp(jerk_weight, 0.0f, 0.50f);
  planner_.set_optimizer_config(opt_cfg);
}

void RobotCoordinator::get_optimizer_params(float& max_lat_dev_mm, float& jerk_weight) const {
  const auto& opt_cfg = planner_.get_optimizer_config();
  max_lat_dev_mm = opt_cfg.max_lateral_deviation_mm;
  jerk_weight = opt_cfg.jerk_damping_weight;
}

void RobotCoordinator::apply_motor_drive(float v_target, float steer, MotorCmd& out) const {
  const float max_speed = config_.profiler.max_straight_speed_mps;

  float vl = v_target + steer;
  float vr = v_target - steer;

  // Scale down velocities proportionally if either wheel exceeds saturation limit,
  // strictly preserving the desired differential turning curvature
  const float peak = std::max(std::abs(vl), std::abs(vr));
  if (peak > max_speed && peak > 0.001f) {
    const float scale = max_speed / peak;
    vl *= scale;
    vr *= scale;
  }

  out.left_pwm = DifferentialDriveKinematics::velocity_to_pwm(vl, max_speed);
  out.right_pwm = DifferentialDriveKinematics::velocity_to_pwm(vr, max_speed);
  out.active_brake = (std::abs(v_target) < 0.001f && std::abs(steer) < 0.001f);
}

}  // namespace lf
