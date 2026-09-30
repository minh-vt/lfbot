#include "control/trajectory_planner.hpp"

#include <algorithm>
#include <cmath>

namespace lf {

namespace {

size_t find_closest_path_point(const std::vector<TrajectoryPoint>& path,
                               const ObstacleGeometry& obs, float& out_min_dist) {
  size_t min_idx = 0;
  float min_dist_sq = 1e12f;
  for (size_t i = 0; i < path.size(); ++i) {
    const float dx = path.at(i).x_mm - obs.x_mm;
    const float dy = path.at(i).y_mm - obs.y_mm;
    const float dist_sq = dx * dx + dy * dy;
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      min_idx = i;
    }
  }
  out_min_dist = std::sqrt(min_dist_sq);
  return min_idx;
}

void apply_obstacle_chicane(std::vector<TrajectoryPoint>& path, const ObstacleGeometry& obs,
                            float clearance_req_mm) {
  if (path.size() < 10) {
    return;
  }

  float min_dist = 0.0f;
  const size_t min_idx = find_closest_path_point(path, obs, min_dist);
  if (min_dist >= clearance_req_mm) {
    return;
  }

  float norm_x = path.at(min_idx).x_mm - obs.x_mm;
  float norm_y = path.at(min_idx).y_mm - obs.y_mm;
  float norm_len = std::sqrt(norm_x * norm_x + norm_y * norm_y);

  if (norm_len < 1.0f) {
    const size_t prev_i = (min_idx > 0) ? (min_idx - 1) : 0;
    const size_t next_i = (min_idx + 1 < path.size()) ? (min_idx + 1) : min_idx;
    const float fwd_x = path.at(next_i).x_mm - path.at(prev_i).x_mm;
    const float fwd_y = path.at(next_i).y_mm - path.at(prev_i).y_mm;
    norm_x = -fwd_y;
    norm_y = fwd_x;
    norm_len = std::sqrt(norm_x * norm_x + norm_y * norm_y);
  }

  if (norm_len > 1e-4f) {
    norm_x /= norm_len;
    norm_y /= norm_len;
  } else {
    norm_x = 0.0f;
    norm_y = 1.0f;
  }

  const float y_max_mm = (clearance_req_mm - min_dist) + 20.0f;
  constexpr int kHalfSpan = 35;
  const int start_i = std::max(0, static_cast<int>(min_idx) - kHalfSpan);
  const int end_i =
    std::min(static_cast<int>(path.size()) - 1, static_cast<int>(min_idx) + kHalfSpan);
  const int span_len = end_i - start_i;

  if (span_len <= 5) {
    return;
  }

  for (int i = start_i; i <= end_i; ++i) {
    const float u = static_cast<float>(i - start_i) / static_cast<float>(span_len);
    const float bump = TrajectoryPlanner::quintic_bump(u);
    path.at(static_cast<size_t>(i)).x_mm += norm_x * y_max_mm * bump;
    path.at(static_cast<size_t>(i)).y_mm += norm_y * y_max_mm * bump;
  }
}

}  // namespace

TrajectoryPlanner::TrajectoryPlanner() : env_(EnvironmentConfig{}), profiler_(ProfilerConfig{}) {}

TrajectoryPlanner::TrajectoryPlanner(const EnvironmentConfig& env, const ProfilerConfig& profiler)
  : env_(env), profiler_(profiler) {}

float TrajectoryPlanner::quintic_step(float u) {
  const float cu = std::clamp(u, 0.0f, 1.0f);
  return cu * cu * cu * (10.0f + cu * (-15.0f + 6.0f * cu));
}

float TrajectoryPlanner::quintic_bump(float u) {
  const float cu = std::clamp(u, 0.0f, 1.0f);
  const float t = cu * (1.0f - cu);
  return 64.0f * t * t * t;
}

void TrajectoryPlanner::profile_speeds(std::vector<TrajectoryPoint>& path) const {
  const size_t n = path.size();
  if (n < 3) {
    for (auto& pt : path) {
      pt.target_speed_mps = profiler_.safe_map_speed_mps;
      pt.curvature_rad_m = 0.0f;
    }
    return;
  }

  const float mu = profiler_.corner_grip_mu;
  const float g = profiler_.gravity_mps2;
  const float a_accel = profiler_.max_accel_mps2;
  const float a_brake = profiler_.max_brake_mps2;
  const float v_top = profiler_.max_straight_speed_mps;
  const float v_safe = profiler_.safe_map_speed_mps;

  // Buoc 1: Tinh do cong Menger va tran toc do ma sat tren tung waypoint
  path.front().curvature_rad_m = 0.0f;
  path.front().target_speed_mps = std::max(profiler_.launch_min_speed_mps, v_safe);
  path.back().curvature_rad_m = 0.0f;
  path.back().target_speed_mps = v_safe;

  for (size_t i = 1; i + 1 < n; ++i) {
    const auto& p_prev = path.at(i - 1);
    const auto& p_curr = path.at(i);
    const auto& p_next = path.at(i + 1);

    const float dx1 = p_curr.x_mm - p_prev.x_mm;
    const float dy1 = p_curr.y_mm - p_prev.y_mm;
    const float dx2 = p_next.x_mm - p_curr.x_mm;
    const float dy2 = p_next.y_mm - p_curr.y_mm;

    const float d1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
    const float d2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
    const float dx3 = p_next.x_mm - p_prev.x_mm;
    const float dy3 = p_next.y_mm - p_prev.y_mm;
    const float d3 = std::sqrt(dx3 * dx3 + dy3 * dy3);

    float curvature = 0.0f;
    const float cross_prod = dx1 * dy2 - dy1 * dx2;
    const float denom = d1 * d2 * d3;
    if (denom > 1e-4f) {
      curvature = (2.0f * std::abs(cross_prod) / denom) * 1000.0f;
    }
    curvature = std::clamp(curvature, 0.0f, 25.0f);
    path.at(i).curvature_rad_m = curvature;

    const float r_effective = (curvature > 0.05f) ? (1.0f / curvature) : 20.0f;
    const float v_grip = std::sqrt(mu * g * r_effective);
    path.at(i).target_speed_mps = std::clamp(v_grip, v_safe, v_top);
  }

  // Buoc 2: Backward Pass (Phanh som truoc cua gat)
  for (size_t i = n - 1; i > 0; --i) {
    const auto& curr = path.at(i);
    auto& prev = path.at(i - 1);
    const float dx = (curr.x_mm - prev.x_mm) * 0.001f;
    const float dy = (curr.y_mm - prev.y_mm) * 0.001f;
    const float ds = std::sqrt(dx * dx + dy * dy);

    const float v_exit = curr.target_speed_mps;
    const float v_entry_limit = std::sqrt(v_exit * v_exit + 2.0f * a_brake * ds);
    prev.target_speed_mps = std::min(prev.target_speed_mps, v_entry_limit);
  }

  // Buoc 3: Forward Pass (Tang toc thoa man gia toc toi da, triet tieu stall tai s = 0)
  path.front().target_speed_mps =
    std::max(profiler_.launch_min_speed_mps, path.front().target_speed_mps);
  for (size_t i = 0; i + 1 < n; ++i) {
    const auto& curr = path.at(i);
    auto& next = path.at(i + 1);
    const float dx = (next.x_mm - curr.x_mm) * 0.001f;
    const float dy = (next.y_mm - curr.y_mm) * 0.001f;
    const float ds = std::sqrt(dx * dx + dy * dy);

    const float v_entry = curr.target_speed_mps;
    const float v_accel_limit = std::sqrt(v_entry * v_entry + 2.0f * a_accel * ds);
    next.target_speed_mps = std::min(next.target_speed_mps, v_accel_limit);
  }
}

std::vector<TrajectoryPoint> TrajectoryPlanner::generate_racing_line(
  const std::vector<TrajectoryPoint>& raw_path,
  const std::vector<ObstacleGeometry>& obstacles) const {
  if (raw_path.empty()) {
    return {};
  }

  std::vector<TrajectoryPoint> optimized = raw_path;
  const float clearance_req_mm = env_.clearance_radius_mm();

  for (const auto& obs : obstacles) {
    apply_obstacle_chicane(optimized, obs, clearance_req_mm);
  }

  profile_speeds(optimized);
  return optimized;
}

}  // namespace lf
