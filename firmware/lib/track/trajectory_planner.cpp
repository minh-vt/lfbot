#include "track/trajectory_planner.hpp"

#include <algorithm>
#include <cmath>

namespace lf {

namespace {

constexpr float kMaxCurvatureRadM = 25.0f;
constexpr float kCorridorMaxDeviationMm = 15.0f;

}  // namespace

TrajectoryPlanner::TrajectoryPlanner() : env_(EnvironmentConfig{}), profiler_(ProfilerConfig{}) {}

TrajectoryPlanner::TrajectoryPlanner(const EnvironmentConfig& env, const ProfilerConfig& profiler)
  : env_(env), profiler_(profiler) {}

void TrajectoryPlanner::profile_speeds(std::vector<TrajectoryPoint>& path,
                                       const std::vector<ObstacleGeometry>& obstacles) const {
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
  const float mu_g = mu * g;

  // Step 1: Calculate Menger curvature and friction grip velocity limit
  path.front().curvature_rad_m = 0.0f;
  path.front().target_speed_mps = std::max(profiler_.launch_min_speed_mps, v_safe);
  path.back().curvature_rad_m = 0.0f;
  path.back().target_speed_mps = v_safe;

  for (size_t i = 1; i + 1 < n; ++i) {
    const auto& p_prev = path.at(i - 1);
    const auto& p_curr = path.at(i);
    const auto& p_next = path.at(i + 1);

    const float raw_k = menger_curvature_rad_m(p_prev.x_mm, p_prev.y_mm, p_curr.x_mm, p_curr.y_mm,
                                               p_next.x_mm, p_next.y_mm, true);
    const float curvature = std::clamp(raw_k, -kMaxCurvatureRadM, kMaxCurvatureRadM);
    path.at(i).curvature_rad_m = curvature;

    // Cornering grip speed: v_grip = sqrt(mu * g / max(|curvature|, epsilon))
    float v_grip = v_top;
    const float abs_k = std::abs(curvature);
    if (abs_k > 0.05f) {
      v_grip = std::sqrt(mu_g / abs_k);
    }

    float v_limit = std::min(v_top, v_grip);
    constexpr float kObsSlowZoneRadiusMm = 320.0f;
    constexpr float kMaxSpeedNearObsMps = 1.05f;
    for (const auto& obs : obstacles) {
      const float dx = p_curr.x_mm - obs.x_mm;
      const float dy = p_curr.y_mm - obs.y_mm;
      if (std::hypot(dx, dy) < kObsSlowZoneRadiusMm) {
        v_limit = std::min(v_limit, kMaxSpeedNearObsMps);
      }
    }

    path.at(i).target_speed_mps = v_limit;
  }

  // Step 2: Backward Pass (Braking limit before corners with Kamm's friction circle)
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

  // Step 3: Forward Pass (Motor acceleration limit from start)
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

  // Global Convex QP Optimization (Seam 1)
  OptimizationConfig opt_cfg{};
  opt_cfg.max_lateral_deviation_mm = kCorridorMaxDeviationMm;
  opt_cfg.obstacle_margin_mm = 30.0f;
  const GlobalTrajectoryOptimizer optimizer(opt_cfg);
  auto optimized = optimizer.optimize(raw_path, obstacles, env_);

  // Time-Optimal Forward-Backward Dynamic Programming Velocity Profiling (Seam 2)
  profile_speeds(optimized, obstacles);

  return optimized;
}

}  // namespace lf
