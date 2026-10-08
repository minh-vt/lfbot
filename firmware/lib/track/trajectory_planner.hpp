#pragma once

#include <vector>

#include "config/robot_config.hpp"
#include "config/types.hpp"
#include "track/global_trajectory_optimizer.hpp"

namespace lf {

class TrajectoryPlanner {
public:
  TrajectoryPlanner();
  explicit TrajectoryPlanner(const EnvironmentConfig& env, const ProfilerConfig& profiler,
                             const OptimizationConfig& opt_cfg = OptimizationConfig{});

  // Generates globally optimized racing line:
  // 1. Minimum-Curvature Convex Quadratic Program (GlobalTrajectoryOptimizer)
  // 2. Continuous corridor box constraints & obstacle clearance
  // 3. Time-optimal forward-backward dynamic programming velocity profiling
  [[nodiscard]] std::vector<TrajectoryPoint> generate_racing_line(
    const std::vector<TrajectoryPoint>& raw_path,
    const std::vector<ObstacleGeometry>& obstacles) const;

  // Calculates friction-limited speed profile over continuous trajectory points
  void profile_speeds(std::vector<TrajectoryPoint>& path,
                      const std::vector<ObstacleGeometry>& obstacles = {}) const;

  [[nodiscard]] const EnvironmentConfig& get_env() const { return env_; }
  [[nodiscard]] const ProfilerConfig& get_profiler() const { return profiler_; }
  [[nodiscard]] const OptimizationConfig& get_optimizer_config() const { return opt_cfg_; }
  void set_optimizer_config(const OptimizationConfig& opt_cfg) { opt_cfg_ = opt_cfg; }

private:
  EnvironmentConfig env_;
  ProfilerConfig profiler_;
  OptimizationConfig opt_cfg_;
};

}  // namespace lf
