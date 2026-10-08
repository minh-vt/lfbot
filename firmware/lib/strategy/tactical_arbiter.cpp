#include "strategy/tactical_arbiter.hpp"

#include <algorithm>
#include <cmath>

#include "config/constants.hpp"

namespace lf {

TacticalBehaviorArbiter::TacticalBehaviorArbiter(const ObstacleConfig& obs_cfg,
                                                 const ProfilerConfig& prof_cfg,
                                                 const TrackingConfig& track_cfg)
  : obs_cfg_(obs_cfg), prof_cfg_(prof_cfg), track_cfg_(track_cfg) {
  reset();
}

void TacticalBehaviorArbiter::reset() {
  line_loss_duration_s_ = 0.0f;
  last_valid_lateral_error_m_ = 0.0f;
  is_evading_ = false;
  evasion_origin_x_ = 0.0f;
  evasion_origin_y_ = 0.0f;
  evasion_origin_th_ = 0.0f;
  evasion_side_sign_ = 1.0f;
  current_bypass_length_m_ = obs_cfg_.bypass_length_m;
  current_bypass_speed_mps_ = obs_cfg_.bypass_speed_mps;
  recovery_timer_s_ = 0.0f;
}

void TacticalBehaviorArbiter::trigger_obstacle_evasion(const Pose2D& pose, float tof_front_m,
                                                       float tof_left_m, float tof_right_m) {
  const bool obstacle_detected =
    (tof_front_m > obs_cfg_.min_valid_tof_m && tof_front_m < obs_cfg_.front_detect_dist_m);

  if (is_evading_ || !obstacle_detected) {
    return;
  }

  is_evading_ = true;
  evasion_origin_x_ = pose.x_m;
  evasion_origin_y_ = pose.y_m;
  evasion_origin_th_ = pose.heading_rad;
  // Direction bias: swerve right if left side is restricted and right is clearer
  evasion_side_sign_ =
    (tof_left_m < obs_cfg_.danger_zone_dist_m && tof_right_m > tof_left_m) ? -1.0f : 1.0f;

  const float d_detect = tof_front_m;
  if (d_detect < obs_cfg_.danger_zone_dist_m) {
    current_bypass_length_m_ =
      std::clamp(2.5f * (d_detect + 0.05f), 0.45f, obs_cfg_.bypass_length_m);
    current_bypass_speed_mps_ = obs_cfg_.recovery_speed_mps;
  } else {
    current_bypass_length_m_ = obs_cfg_.bypass_length_m;
    current_bypass_speed_mps_ = obs_cfg_.bypass_speed_mps;
  }
}

TacticalReference TacticalBehaviorArbiter::update(const Pose2D& pose, float /*current_v_mps*/,
                                                  float centroid_mm, bool line_found,
                                                  float tof_front_m, float tof_left_m,
                                                  float tof_right_m, float dt, RobotMode mode) {
  TacticalReference ref{};

  if (recovery_timer_s_ > 0.0f) {
    recovery_timer_s_ = std::max(0.0f, recovery_timer_s_ - dt);
  }

  // 1. Check obstacle presence in Run 1 Map mode
  if (mode == RobotMode::kRun1Map) {
    trigger_obstacle_evasion(pose, tof_front_m, tof_left_m, tof_right_m);
  }

  // 2. Continuous C2 Raised-Cosine Chicane Evasion
  if (is_evading_) {
    const float dx = pose.x_m - evasion_origin_x_;
    const float dy = pose.y_m - evasion_origin_y_;
    const float cos_th0 = std::cos(evasion_origin_th_);
    const float sin_th0 = std::sin(evasion_origin_th_);

    const float s = dx * cos_th0 + dy * sin_th0;
    const float d_left = dx * sin_th0 - dy * cos_th0;
    const float d_actual = d_left * evasion_side_sign_;

    const float l_bypass = current_bypass_length_m_;
    const float y_max = obs_cfg_.bypass_clearance_m;

    if (s >= l_bypass) {
      is_evading_ = false;
      recovery_timer_s_ = 0.25f;
    } else {
      const float s_clamped = std::clamp(s, 0.0f, l_bypass);
      const float u = (constants::kTwoPi * s_clamped) / l_bypass;
      const float d_target = 0.5f * y_max * (1.0f - std::cos(u));
      const float d_slope = (constants::kPi * y_max / l_bypass) * std::sin(u);
      const float psi_target = evasion_origin_th_ - evasion_side_sign_ * std::atan(d_slope);

      const float cross_track_err = d_target - d_actual;
      const float heading_err = std::remainder(pose.heading_rad - psi_target, constants::kTwoPi);

      constexpr float kHeadingGain = 1.20f;
      constexpr float kCrossTrackGain = 1.80f;
      const float v_target = current_bypass_speed_mps_;

      const float steer = -(kHeadingGain * heading_err +
                            std::atan2(kCrossTrackGain * cross_track_err, v_target + 0.10f));

      ref.target_v_mps = v_target;
      ref.steer_cmd_rad = steer;
      ref.has_steer_override = true;
      ref.is_evading = true;
      ref.bypass_state = 1;
      ref.should_halt = false;
      return ref;
    }
  }

  // 3. Post-evasion active recovery if line is not yet reacquired
  if (!line_found && recovery_timer_s_ > 0.0f) {
    ref.target_v_mps = obs_cfg_.recovery_speed_mps;
    ref.steer_cmd_rad = evasion_side_sign_ * obs_cfg_.recovery_steer_rad;
    ref.has_steer_override = true;
    ref.is_evading = false;
    ref.bypass_state = 0;
    ref.should_halt = false;
    return ref;
  }

  // 4. Optical Line Presence & Dead-Reckoning
  const float optical_lateral_err_m = centroid_mm * 0.001f;

  if (line_found) {
    line_loss_duration_s_ = 0.0f;
    last_valid_lateral_error_m_ = optical_lateral_err_m;
    recovery_timer_s_ = 0.0f;
    ref.is_dead_reckoning = false;
    ref.should_halt = false;
    ref.lateral_error_m = optical_lateral_err_m;

    // Ancient Method (All-Japan RoboTrace Optical Dynamic Braking):
    // Continuous dynamic deceleration based on lateral centroid displacement.
    // Straight track (centroid ~ 0) runs at full safe_map_speed.
    // As displacement grows on sharp corners/90-deg bends (|e| ~ 20-30mm),
    // target velocity continuously scales down to safe cornering crawling speed.
    const float err_ratio = std::abs(centroid_mm) / 20.0f;
    ref.target_v_mps = prof_cfg_.safe_map_speed_mps / (1.0f + err_ratio * err_ratio);
    ref.has_steer_override = false;
    ref.bypass_state = 0;
  } else {
    line_loss_duration_s_ += dt;

    if (mode == RobotMode::kRun2Sprint) {
      ref.is_dead_reckoning = false;
      ref.should_halt = false;
      ref.lateral_error_m = 0.0f;
      ref.target_v_mps = prof_cfg_.max_straight_speed_mps;
      ref.has_steer_override = false;
      ref.bypass_state = 0;
    } else if (line_loss_duration_s_ < track_cfg_.line_loss_timeout_s) {
      // 300ms Dead-Reckoning Momentum: Maintain course across gaps/markers
      ref.is_dead_reckoning = true;
      ref.should_halt = false;
      ref.lateral_error_m = last_valid_lateral_error_m_;
      ref.target_v_mps = prof_cfg_.safe_map_speed_mps;
      ref.has_steer_override = false;
      ref.bypass_state = 0;
    } else {
      // Safe halt beyond timeout
      ref.should_halt = true;
      ref.target_v_mps = 0.0f;
      ref.lateral_error_m = 0.0f;
      ref.has_steer_override = false;
      ref.bypass_state = 0;
    }
  }

  return ref;
}

}  // namespace lf
