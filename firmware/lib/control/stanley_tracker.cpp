#include "control/stanley_tracker.hpp"

#include <algorithm>
#include <cmath>

#include "config/constants.hpp"

namespace lf {

namespace {

size_t find_closest_waypoint(const std::vector<TrajectoryPoint>& trajectory, float x_m, float y_m,
                             size_t start_idx, size_t end_idx) {
  size_t best_idx = start_idx;
  float min_dist_sq = 1e12f;
  for (size_t i = start_idx; i < end_idx; ++i) {
    const float dx = x_m - trajectory.at(i).x_mm * 0.001f;
    const float dy = y_m - trajectory.at(i).y_mm * 0.001f;
    const float d2 = dx * dx + dy * dy;
    if (d2 < min_dist_sq) {
      min_dist_sq = d2;
      best_idx = i;
    }
  }
  return best_idx;
}

}  // namespace

StanleyTracker::StanleyTracker() : config_(StanleyConfig{}) {}

StanleyTracker::StanleyTracker(const StanleyConfig& config) : config_(config) {}

float StanleyTracker::normalize_angle(float angle_rad) {
  return std::remainder(angle_rad, constants::kTwoPi);
}

TrackingCommand StanleyTracker::update(const Pose2D& pose, float current_v_mps,
                                       const std::vector<TrajectoryPoint>& trajectory,
                                       size_t hint_idx) const {
  TrackingCommand cmd{};
  if (trajectory.empty()) {
    return cmd;
  }

  // 1. Find closest waypoint to robot pose (vehicle center)
  // When hint_idx is 0, acquire globally. Once tracking, use forward-monotonic window.
  const size_t search_start = (hint_idx > 5) ? (hint_idx - 5) : 0;
  const size_t search_end =
    (hint_idx == 0) ? trajectory.size() : std::min(trajectory.size(), hint_idx + 40);
  const size_t robot_idx =
    find_closest_waypoint(trajectory, pose.x_m, pose.y_m, search_start, search_end);

  cmd.closest_waypoint_idx = static_cast<int>(robot_idx);
  cmd.target_v_mps = trajectory.at(robot_idx).target_speed_mps;

  // 2. Front axle / lookahead reference point for steering law
  const float cos_th = std::cos(pose.heading_rad);
  const float sin_th = std::sin(pose.heading_rad);
  const float fx_m = pose.x_m + config_.lookahead_dist_m * cos_th;
  const float fy_m = pose.y_m + config_.lookahead_dist_m * sin_th;

  // Find closest waypoint to front axle in a forward window
  const size_t steer_search_start = (robot_idx > 5) ? (robot_idx - 5) : 0;
  const size_t steer_search_end = std::min(trajectory.size(), robot_idx + 30);
  const size_t steer_idx =
    find_closest_waypoint(trajectory, fx_m, fy_m, steer_search_start, steer_search_end);

  const auto& target_pt = trajectory.at(steer_idx);

  // 3. Heading Error psi_e
  const float path_heading = target_pt.heading_rad;
  const float heading_error = normalize_angle(path_heading - pose.heading_rad);
  cmd.heading_error_rad = heading_error;

  const float dx = fx_m - target_pt.x_mm * 0.001f;
  const float dy = fy_m - target_pt.y_mm * 0.001f;
  const float nx = -std::sin(path_heading);
  const float ny = std::cos(path_heading);
  const float cross_track_error = dx * nx + dy * ny;
  cmd.cross_track_error_m = cross_track_error;

  // 5. Stanley Steering Law:
  // delta = k_heading * psi_e + atan2(-k_cross * e, v + k_soft) + feedforward_curvature
  const float v_effective = std::max(current_v_mps, 0.05f);
  const float lateral_term =
    std::atan2(-config_.k_cross_track * cross_track_error, v_effective + config_.k_soft);
  const float feedforward_steer = 0.5f * config_.wheel_base_m * target_pt.curvature_rad_m;

  float delta = config_.k_heading * heading_error + lateral_term + feedforward_steer;
  delta = std::clamp(delta, -config_.max_steer_rad, config_.max_steer_rad);
  cmd.steering_angle_rad = delta;

  // 6. Wheel Speed Allocation (Differential Drive)
  const float target_v = cmd.target_v_mps;
  float vl = target_v * (1.0f + delta);
  float vr = target_v * (1.0f - delta);

  if (target_v > 0.0f) {
    vl = std::max(0.0f, vl);
    vr = std::max(0.0f, vr);
  }

  cmd.left_wheel_mps = vl;
  cmd.right_wheel_mps = vr;

  return cmd;
}

}  // namespace lf
