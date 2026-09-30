#include "system/robot_coordinator.hpp"

#include <algorithm>
#include <cmath>

#include "math/kinematics.hpp"

namespace lf {

RobotCoordinator::RobotCoordinator(const RobotConfig& config)
  : config_(config),
    centroid_calc_(config.line_sensor),
    odometry_(config.kinematics),
    line_pid_(config.line_pid),
    speed_pid_l_(config.speed_pid_l),
    speed_pid_r_(config.speed_pid_r),
    obstacle_fsm_(config.obstacle),
    profiler_(config.profiler) {
  init();
}

void RobotCoordinator::init() {
  mode_ = RobotMode::kIdle;
  telemetry_ = {};
  segment_start_dist_m_ = 0.0f;
  current_segment_type_ = SegmentType::kStraight;
  finish_entry_dist_m_ = -1.0f;

  odometry_.reset();
  line_pid_.reset();
  speed_pid_l_.reset();
  speed_pid_r_.reset();
  obstacle_fsm_.reset();
  profiler_.reset();
}

void RobotCoordinator::reset_pose() {
  mode_ = RobotMode::kIdle;
  telemetry_ = {};
  segment_start_dist_m_ = 0.0f;
  current_segment_type_ = SegmentType::kStraight;
  finish_entry_dist_m_ = -1.0f;

  odometry_.reset();
  line_pid_.reset();
  speed_pid_l_.reset();
  speed_pid_r_.reset();
  obstacle_fsm_.reset();
  // Preserve profiler_ to maintain learned track map
}

void RobotCoordinator::set_mode(RobotMode mode) {
  mode_ = mode;
  line_pid_.reset();
  speed_pid_l_.reset();
  speed_pid_r_.reset();
  obstacle_fsm_.reset();
  finish_entry_dist_m_ = -1.0f;

  if (mode == RobotMode::kRun1Map) {
    profiler_.reset();
    odometry_.reset();
    segment_start_dist_m_ = 0.0f;
    current_segment_type_ = SegmentType::kStraight;
    return;
  }

  if (mode == RobotMode::kRun2Sprint) {
    odometry_.reset();
    return;
  }
}

void RobotCoordinator::start_auto_tune() {
  set_mode(RobotMode::kAutoTuning);
  auto_tuner_.start(0.4f, 6);
}

void RobotCoordinator::handle_idle_mode(MotorCmd& out) {
  out.left_pwm = 0.0f;
  out.right_pwm = 0.0f;
  out.active_brake = true;
}

void RobotCoordinator::handle_auto_tune_mode(float centroid_mm, float dt, MotorCmd& out) {
  const float steer_cmd = auto_tuner_.update(centroid_mm, dt);
  out.left_pwm = 0.35f + steer_cmd;
  out.right_pwm = 0.35f - steer_cmd;
  out.active_brake = false;

  if (auto_tuner_.get_state() != RelayAutoTuner::State::kConverged) {
    return;
  }

  float tuned_kp = 0.0f;
  float tuned_ki = 0.0f;
  float tuned_kd = 0.0f;
  auto_tuner_.get_tuned_gains(tuned_kp, tuned_ki, tuned_kd);
  set_line_pid_gains(tuned_kp, tuned_ki, tuned_kd);
  set_mode(RobotMode::kIdle);
}

bool RobotCoordinator::check_finish_condition(bool is_finish, float total_dist, MotorCmd& out) {
  if (finish_entry_dist_m_ < 0.0f) {
    if (!is_finish || total_dist <= config_.tracking.min_finish_arm_dist_m) {
      return false;
    }
    finish_entry_dist_m_ = total_dist;

    if (mode_ == RobotMode::kRun1Map) {
      const float seg_len = finish_entry_dist_m_ - segment_start_dist_m_;
      profiler_.add_segment(current_segment_type_, seg_len, 0.0f);
      profiler_.add_segment(SegmentType::kFinishZone, config_.tracking.finish_zone_length_m, 0.0f);
      profiler_.optimize();
    }
  }

  const float dist_in_finish = total_dist - finish_entry_dist_m_;
  if (dist_in_finish < config_.tracking.finish_zone_length_m) {
    const float finish_speed = config_.profiler.safe_map_speed_mps * 0.5f;
    apply_motor_drive(finish_speed, 0.0f, out);
    return true;
  }

  set_mode(RobotMode::kIdle);
  out.left_pwm = 0.0f;
  out.right_pwm = 0.0f;
  out.active_brake = true;
  return true;
}

void RobotCoordinator::apply_motor_drive(float v_target, float steer, MotorCmd& out) const {
  const float max_speed = config_.profiler.max_straight_speed_mps;
  out.left_pwm = DifferentialDriveKinematics::velocity_to_pwm(v_target + steer, max_speed);
  out.right_pwm = DifferentialDriveKinematics::velocity_to_pwm(v_target - steer, max_speed);
  out.active_brake = false;
}

void RobotCoordinator::handle_run_1_map_mode(const SensorData& in, float centroid_mm,
                                             bool line_found, bool is_finish, float dt,
                                             MotorCmd& out) {
  const float total_dist = odometry_.get_total_distance_m();

  if (check_finish_condition(is_finish, total_dist, out)) {
    return;
  }

  obstacle_fsm_.update(in, odometry_.get_current_velocity_mps(), out, line_found);
  if (obstacle_fsm_.is_bypassing()) {
    if (current_segment_type_ != SegmentType::kObstacleZone) {
      const float seg_len = total_dist - segment_start_dist_m_;
      profiler_.add_segment(current_segment_type_, seg_len, 0.0f);
      segment_start_dist_m_ = total_dist;
      current_segment_type_ = SegmentType::kObstacleZone;
    }
    return;
  }

  float v_target = config_.profiler.safe_map_speed_mps;
  if (!line_found) {
    v_target *= config_.tracking.min_reactive_speed_scale;
  } else {
    const float error_factor =
      std::clamp(1.0f - (std::abs(centroid_mm) / config_.tracking.max_error_mm),
                 config_.tracking.min_reactive_speed_scale, 1.0f);
    v_target *= error_factor;
  }

  const float steer = line_pid_.update(centroid_mm, dt);
  apply_motor_drive(v_target, steer, out);

  SegmentType detected_type = SegmentType::kStraight;
  float curvature = 0.0f;
  if (std::abs(centroid_mm) > config_.tracking.curve_centroid_threshold_mm) {
    detected_type = (centroid_mm < 0.0f) ? SegmentType::kCurveLeft : SegmentType::kCurveRight;
    curvature = (std::abs(centroid_mm) / config_.tracking.max_centroid_curvature_mm) *
                config_.tracking.max_curvature_rad_m;
  }

  const bool is_new_segment = (detected_type != current_segment_type_);
  const bool has_min_length =
    (total_dist - segment_start_dist_m_ > config_.tracking.min_segment_length_m);

  if (is_new_segment && has_min_length) {
    const float seg_len = total_dist - segment_start_dist_m_;
    profiler_.add_segment(current_segment_type_, seg_len, curvature);
    segment_start_dist_m_ = total_dist;
    current_segment_type_ = detected_type;
  }
}

void RobotCoordinator::handle_run_2_sprint_mode(const SensorData& in, float centroid_mm,
                                                bool line_found, bool is_finish, float dt,
                                                MotorCmd& out) {
  const float total_dist = odometry_.get_total_distance_m();

  if (check_finish_condition(is_finish, total_dist, out)) {
    return;
  }

  obstacle_fsm_.update(in, odometry_.get_current_velocity_mps(), out, line_found);
  if (obstacle_fsm_.is_bypassing()) {
    return;
  }

  float v_target = config_.profiler.safe_map_speed_mps;
  float ff_steer = 0.0f;
  int seg_idx = 0;

  if (profiler_.is_optimized()) {
    profiler_.query(total_dist, v_target, ff_steer, seg_idx);
    if (finish_entry_dist_m_ < 0.0f) {
      const float min_approach_speed = config_.profiler.safe_map_speed_mps * 0.5f;
      v_target = std::max(v_target, min_approach_speed);
    }
  } else {
    const float error_factor =
      std::clamp(1.0f - (std::abs(centroid_mm) / config_.tracking.max_error_mm),
                 config_.tracking.min_reactive_speed_scale, 1.0f);
    v_target = config_.profiler.max_straight_speed_mps * error_factor;
  }

  if (!line_found) {
    v_target *= config_.tracking.min_reactive_speed_scale;
  }

  telemetry_.current_segment_idx = seg_idx;
  telemetry_.current_velocity_mps = v_target;

  const float steer = line_pid_.update(centroid_mm, dt) + ff_steer;
  apply_motor_drive(v_target, steer, out);
}

void RobotCoordinator::update(const SensorData& in, MotorCmd& out) {
  const float dt =
    (in.dt > config_.tracking.min_valid_dt_s) ? in.dt : config_.tracking.default_loop_dt_s;
  odometry_.update(in.enc_left_ticks, in.enc_right_ticks, dt);

  float centroid_mm = telemetry_.centroid_pos_mm;
  const bool line_found = centroid_calc_.compute(in.ir_raw, centroid_mm);
  const bool is_finish = centroid_calc_.is_finish_line(in.ir_raw);

  if (line_found) {
    telemetry_.centroid_pos_mm = centroid_mm;
  } else {
    // Line-loss Memory Latch: Khi mat vach, be lai cuc dai ve huong vach vua thoat ra
    // Neu vach vua mat o ben phai (centroid > 0) -> ep sai so +max_error de re phai gat
    // Neu vach vua mat o ben trai (centroid < 0) -> ep sai so -max_error de re trai gat
    centroid_mm = (telemetry_.centroid_pos_mm >= 0.0f) ? config_.tracking.max_error_mm
                                                       : -config_.tracking.max_error_mm;
  }

  telemetry_.line_lost = !line_found;
  telemetry_.total_distance_m = odometry_.get_total_distance_m();
  telemetry_.current_velocity_mps = odometry_.get_current_velocity_mps();
  telemetry_.mode = mode_;
  telemetry_.fsm_state = static_cast<uint8_t>(obstacle_fsm_.get_state());

  if (mode_ == RobotMode::kIdle || mode_ == RobotMode::kEmergencyStop) {
    handle_idle_mode(out);
  } else if (mode_ == RobotMode::kAutoTuning) {
    handle_auto_tune_mode(centroid_mm, dt, out);
  } else if (mode_ == RobotMode::kRun1Map) {
    handle_run_1_map_mode(in, centroid_mm, line_found, is_finish, dt, out);
  } else if (mode_ == RobotMode::kRun2Sprint) {
    handle_run_2_sprint_mode(in, centroid_mm, line_found, is_finish, dt, out);
  }

  // Cap nhat Telemetry chi tiet ve dong co va cam bien vat can ToF
  telemetry_.wheel_l.enc_ticks = in.enc_left_ticks;
  telemetry_.wheel_r.enc_ticks = in.enc_right_ticks;
  telemetry_.wheel_l.actual_speed_mps = odometry_.get_left_velocity_mps();
  telemetry_.wheel_r.actual_speed_mps = odometry_.get_right_velocity_mps();
  telemetry_.wheel_l.pwm_duty = out.left_pwm;
  telemetry_.wheel_r.pwm_duty = out.right_pwm;
  telemetry_.obstacle.tof_front_m = in.tof_front_m;
  telemetry_.obstacle.tof_left_m = in.tof_left_m;
  telemetry_.obstacle.tof_right_m = in.tof_right_m;
  telemetry_.obstacle.nearest_dist_m = std::min({in.tof_front_m, in.tof_left_m, in.tof_right_m});
  telemetry_.obstacle.detected =
    obstacle_fsm_.is_bypassing() || (in.tof_front_m < config_.obstacle.front_detect_dist_m);
}

void RobotCoordinator::set_line_pid_gains(float kp, float ki, float kd) {
  config_.line_pid.kp = std::clamp(kp, 0.005f, 0.200f);
  config_.line_pid.ki = std::clamp(ki, 0.0f, 0.050f);
  config_.line_pid.kd = std::clamp(kd, 0.0005f, 0.050f);
  line_pid_.set_gains(config_.line_pid.kp, config_.line_pid.ki, config_.line_pid.kd);
}

void RobotCoordinator::get_line_pid_gains(float& kp, float& ki, float& kd) const {
  kp = config_.line_pid.kp;
  ki = config_.line_pid.ki;
  kd = config_.line_pid.kd;
}

void RobotCoordinator::set_tactical_speeds(float safe_map_speed, float max_straight_speed) {
  config_.profiler.safe_map_speed_mps = std::clamp(safe_map_speed, 0.20f, 1.50f);
  config_.profiler.max_straight_speed_mps = std::clamp(max_straight_speed, 0.50f, 4.00f);
  profiler_.set_config(config_.profiler);
}

void RobotCoordinator::set_profiler_dynamics(float max_accel, float max_brake, float corner_grip) {
  config_.profiler.max_accel_mps2 = std::clamp(max_accel, 1.0f, 6.0f);
  config_.profiler.max_brake_mps2 = std::clamp(max_brake, 1.5f, 8.0f);
  config_.profiler.corner_grip_mu = std::clamp(corner_grip, 0.3f, 1.2f);
  profiler_.set_config(config_.profiler);
}

void RobotCoordinator::set_obstacle_distances(float front_detect_dist_m,
                                              float approach_stop_dist_m) {
  config_.obstacle.front_detect_dist_m = std::clamp(front_detect_dist_m, 0.08f, 0.80f);
  config_.obstacle.approach_stop_dist_m =
    std::clamp(approach_stop_dist_m, 0.03f, config_.obstacle.front_detect_dist_m - 0.02f);
  obstacle_fsm_.set_config(config_.obstacle);
}

void RobotCoordinator::get_obstacle_distances(float& front_detect_dist_m,
                                              float& approach_stop_dist_m) const {
  front_detect_dist_m = config_.obstacle.front_detect_dist_m;
  approach_stop_dist_m = config_.obstacle.approach_stop_dist_m;
}

void RobotCoordinator::set_obstacle_parameters(float front_detect_dist_m, float clearance_m,
                                               float length_m) {
  config_.obstacle.front_detect_dist_m = std::clamp(front_detect_dist_m, 0.10f, 0.80f);
  config_.obstacle.bypass_clearance_m = std::clamp(clearance_m, 0.08f, 0.35f);
  config_.obstacle.bypass_length_m = std::clamp(length_m, 0.30f, 1.50f);
  obstacle_fsm_.set_config(config_.obstacle);
}

void RobotCoordinator::get_obstacle_parameters(float& front_detect_dist_m, float& clearance_m,
                                               float& length_m) const {
  front_detect_dist_m = config_.obstacle.front_detect_dist_m;
  clearance_m = config_.obstacle.bypass_clearance_m;
  length_m = config_.obstacle.bypass_length_m;
}

}  // namespace lf
