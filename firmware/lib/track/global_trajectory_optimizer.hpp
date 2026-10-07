#pragma once

#include <cmath>
#include <vector>

#include "config/robot_config.hpp"
#include "config/types.hpp"

namespace lf {

[[nodiscard]] inline float menger_curvature_rad_m(float x1, float y1, float x2, float y2, float x3,
                                                  float y3, bool is_signed = false) {
  const float dx1 = x2 - x1;
  const float dy1 = y2 - y1;
  const float dx2 = x3 - x2;
  const float dy2 = y3 - y2;

  const float d1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
  const float d2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
  const float dx3 = x3 - x1;
  const float dy3 = y3 - y1;
  const float d3 = std::sqrt(dx3 * dx3 + dy3 * dy3);

  const float cross = dx1 * dy2 - dy1 * dx2;
  const float denom = d1 * d2 * d3;
  if (denom <= 1e-6f) {
    return 0.0f;
  }
  return (2.0f * (is_signed ? cross : std::abs(cross)) / denom) * 1000.0f;
}

struct OptimizationConfig {
  float max_lateral_deviation_mm = 15.0f;
  float curvature_weight = 1.0f;
  float reg_weight = 1e-4f;
  float jerk_damping_weight = 0.05f;
  int max_iterations = 250;
  float tolerance_mm = 0.01f;
  float obstacle_margin_mm = 15.0f;
  float relevance_margin_mm = 150.0f;
  float min_transition_length_mm = 400.0f;
  float transition_factor = 2.5f;
};

class GlobalTrajectoryOptimizer {
public:
  GlobalTrajectoryOptimizer();
  explicit GlobalTrajectoryOptimizer(const OptimizationConfig& config);

  [[nodiscard]] std::vector<TrajectoryPoint> optimize(
    const std::vector<TrajectoryPoint>& centerline, const std::vector<ObstacleGeometry>& obstacles,
    const EnvironmentConfig& env) const;

  [[nodiscard]] const OptimizationConfig& get_config() const { return config_; }

private:
  OptimizationConfig config_;
};

}  // namespace lf
