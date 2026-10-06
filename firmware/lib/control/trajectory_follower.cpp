#include "control/trajectory_follower.hpp"

#include <algorithm>
#include <cmath>

#include "config/constants.hpp"
#include "math/kinematics.hpp"

namespace lf {

TrajectoryFollower::TrajectoryFollower(const PIDConfig& pid_cfg, const ProfilerConfig& prof_cfg,
                                       const ObstacleConfig& obs_cfg)
  : line_pid_(pid_cfg), prof_cfg_(prof_cfg), obs_cfg_(obs_cfg), stanley_(StanleyConfig{}) {}

void TrajectoryFollower::reset() {
  line_pid_.reset();
  clear_trajectory();
  is_evading_ = false;
  evasion_origin_x_ = 0.0f;
  evasion_origin_y_ = 0.0f;
  evasion_origin_th_ = 0.0f;
  evasion_side_sign_ = 1.0f;
  evasion_cooldown_s_ = 0.0f;
  cross_track_error_m_ = 0.0f;
  heading_error_rad_ = 0.0f;
}

void TrajectoryFollower::set_trajectory(const std::vector<TrajectoryPoint>& trajectory) {
  trajectory_ = trajectory;
  last_index_ = 0;
}

void TrajectoryFollower::apply_motor_drive(float v_target, float steer, MotorCmd& out) const {
  const float max_speed = prof_cfg_.max_straight_speed_mps;

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
  out.active_brake = false;
}

void TrajectoryFollower::update(const Pose2D& pose, float current_v_mps, float /*delta_dist_m*/,
                                float centroid_mm, bool line_found, float dt, const SensorData& in,
                                MotorCmd& out, float& out_target_v, float& out_target_steer,
                                int& out_waypoint_idx, uint8_t& out_bypass_state) {
  out_bypass_state = 0;

  // 1. If global racing trajectory is available (Run 2 Sprint):
  // Track using continuous Stanley controller
  if (!trajectory_.empty()) {
    const auto cmd = stanley_.update(pose, current_v_mps, trajectory_, last_index_);
    out_waypoint_idx = cmd.closest_waypoint_idx;
    last_index_ = static_cast<size_t>(std::max(0, cmd.closest_waypoint_idx));
    cross_track_error_m_ = cmd.cross_track_error_m;
    heading_error_rad_ = cmd.heading_error_rad;

    const float max_speed = prof_cfg_.max_straight_speed_mps;
    out.left_pwm = DifferentialDriveKinematics::velocity_to_pwm(cmd.left_wheel_mps, max_speed);
    out.right_pwm = DifferentialDriveKinematics::velocity_to_pwm(cmd.right_wheel_mps, max_speed);
    out.active_brake = false;

    out_target_v = cmd.target_v_mps;
    out_target_steer = cmd.steering_angle_rad;
    return;
  }

  // 2. Exploration mode without trajectory (Run 1 Map):
  if (evasion_cooldown_s_ > 0.0f) {
    evasion_cooldown_s_ = std::max(0.0f, evasion_cooldown_s_ - dt);
  }

  // Continuous C2 Raised-Cosine Chicane Evasion upon obstacle detection:
  const bool obstacle_detected =
    (in.tof_front_m > obs_cfg_.min_valid_tof_m && in.tof_front_m < obs_cfg_.front_detect_dist_m);

  if (!is_evading_ && obstacle_detected && evasion_cooldown_s_ <= 0.0f) {
    is_evading_ = true;
    evasion_origin_x_ = pose.x_m;
    evasion_origin_y_ = pose.y_m;
    evasion_origin_th_ = pose.heading_rad;
    // Default to swerving left unless left space is constricted and right is clearer
    evasion_side_sign_ = (in.tof_left_m < 0.20f && in.tof_right_m > in.tof_left_m) ? -1.0f : 1.0f;
  }

  if (is_evading_) {
    const float dx = pose.x_m - evasion_origin_x_;
    const float dy = pose.y_m - evasion_origin_y_;
    const float cos_th0 = std::cos(evasion_origin_th_);
    const float sin_th0 = std::sin(evasion_origin_th_);

    // Forward progress along initial heading at evasion trigger
    const float s = dx * cos_th0 + dy * sin_th0;

    // Actual lateral displacement (positive = to the left of initial heading)
    const float d_left = dx * sin_th0 - dy * cos_th0;
    const float d_actual = d_left * evasion_side_sign_;

    const float l_bypass = obs_cfg_.bypass_length_m;
    const float y_max = obs_cfg_.bypass_clearance_m;

    // Conclude evasion when continuous C2 chicane completes full trajectory
    if (s >= l_bypass) {
      is_evading_ = false;
      evasion_cooldown_s_ = 0.25f;
    } else {
      // Continuous C2 Raised-Cosine target profile
      const float s_clamped = std::clamp(s, 0.0f, l_bypass);
      const float u = (constants::kTwoPi * s_clamped) / l_bypass;
      const float d_target = 0.5f * y_max * (1.0f - std::cos(u));
      const float d_slope = (constants::kPi * y_max / l_bypass) * std::sin(u);
      const float psi_target = evasion_origin_th_ - evasion_side_sign_ * std::atan(d_slope);

      const float cross_track_err = d_target - d_actual;
      const float heading_err = std::remainder(pose.heading_rad - psi_target, constants::kTwoPi);

      constexpr float kHeadingGain = 1.20f;
      constexpr float kCrossTrackGain = 1.80f;
      const float v_target = obs_cfg_.bypass_speed_mps;

      const float steer = -(kHeadingGain * heading_err +
                            std::atan2(kCrossTrackGain * cross_track_err, v_target + 0.10f));

      apply_motor_drive(v_target, steer, out);
      out_target_v = v_target;
      out_target_steer = steer;
      out_waypoint_idx = 0;
      out_bypass_state = 1;
      return;
    }
  }

  // Safe optical line tracking
  if (!line_found) {
    const float v_target = 0.0f;
    const float steer = 0.0f;
    apply_motor_drive(v_target, steer, out);
    out_target_v = v_target;
    out_target_steer = steer;
    out_waypoint_idx = 0;
    return;
  }

  const float v_target = prof_cfg_.safe_map_speed_mps;
  const float steer = line_pid_.update(centroid_mm, dt);
  apply_motor_drive(v_target, steer, out);
  out_target_v = v_target;
  out_target_steer = steer;
  out_waypoint_idx = 0;
}

}  // namespace lf
