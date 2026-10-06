#include "track/global_trajectory_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "config/constants.hpp"

namespace lf {

namespace {

struct CenterlineGeometry {
  std::vector<float> s_m;
  std::vector<float> ds_m;
  std::vector<float> nx;
  std::vector<float> ny;
  std::vector<float> kappa_cl;
};

CenterlineGeometry compute_centerline_geometry(const std::vector<TrajectoryPoint>& centerline) {
  const size_t n = centerline.size();
  CenterlineGeometry geom{std::vector<float>(n, 0.0f), std::vector<float>(n, 0.0f),
                          std::vector<float>(n, 0.0f), std::vector<float>(n, 0.0f),
                          std::vector<float>(n, 0.0f)};

  for (size_t i = 1; i < n; ++i) {
    const float dist_mm = std::hypot(centerline.at(i).x_mm - centerline.at(i - 1).x_mm,
                                     centerline.at(i).y_mm - centerline.at(i - 1).y_mm);
    geom.ds_m.at(i) = dist_mm * 0.001f;
    geom.s_m.at(i) = geom.s_m.at(i - 1) + geom.ds_m.at(i);
  }

  for (size_t i = 0; i < n; ++i) {
    const size_t prev_i = (i > 0) ? (i - 1) : 0;
    const size_t next_i = (i + 1 < n) ? (i + 1) : (n - 1);
    const float dx = centerline.at(next_i).x_mm - centerline.at(prev_i).x_mm;
    const float dy = centerline.at(next_i).y_mm - centerline.at(prev_i).y_mm;
    const float th = std::atan2(dy, dx);
    geom.nx.at(i) = -std::sin(th);
    geom.ny.at(i) = std::cos(th);
  }

  for (size_t i = 1; i + 1 < n; ++i) {
    geom.kappa_cl.at(i) = menger_curvature_rad_m(
      centerline.at(i - 1).x_mm, centerline.at(i - 1).y_mm, centerline.at(i).x_mm,
      centerline.at(i).y_mm, centerline.at(i + 1).x_mm, centerline.at(i + 1).y_mm, true);
  }
  return geom;
}

void apply_obstacle_corridor_constraints(const std::vector<TrajectoryPoint>& centerline,
                                         const std::vector<ObstacleGeometry>& obstacles,
                                         const EnvironmentConfig& env,
                                         const CenterlineGeometry& geom, float obstacle_margin_mm,
                                         std::vector<float>& alpha_min,
                                         std::vector<float>& alpha_max) {
  const size_t n = centerline.size();
  const float r_clearance_mm = env.clearance_radius_mm() + obstacle_margin_mm;

  for (const auto& obs : obstacles) {
    size_t obs_idx = 0;
    float min_dist_sq = 1e12f;
    for (size_t i = 0; i < n; ++i) {
      const float dx = obs.x_mm - centerline.at(i).x_mm;
      const float dy = obs.y_mm - centerline.at(i).y_mm;
      const float d2 = dx * dx + dy * dy;
      if (d2 < min_dist_sq) {
        min_dist_sq = d2;
        obs_idx = i;
      }
    }

    const float min_dist = std::sqrt(min_dist_sq);
    if (min_dist > r_clearance_mm + 150.0f) {
      continue;
    }

    const float s_obs = geom.s_m.at(obs_idx);
    const float vx = obs.x_mm - centerline.at(obs_idx).x_mm;
    const float vy = obs.y_mm - centerline.at(obs_idx).y_mm;
    const float d_obs = vx * geom.nx.at(obs_idx) + vy * geom.ny.at(obs_idx);

    const float l_trans_mm = std::max(400.0f, 2.5f * r_clearance_mm);

    for (size_t i = 0; i < n; ++i) {
      const float ds_obs_mm = std::abs(geom.s_m.at(i) - s_obs) * 1000.0f;
      if (ds_obs_mm > l_trans_mm) {
        continue;
      }
      const float u = ds_obs_mm / l_trans_mm;
      const float envelope = 0.5f * (1.0f + std::cos(constants::kPi * u));

      if (d_obs <= 0.0f) {
        // Obstacle is on negative normal side (or centered). Pass on positive side (alpha >=
        // bound).
        const float bound = (d_obs + r_clearance_mm) * envelope;
        alpha_min.at(i) = std::max(alpha_min.at(i), bound);
        alpha_max.at(i) = std::max(alpha_max.at(i), bound + obstacle_margin_mm);
      } else {
        // Obstacle is on positive normal side. Pass on negative side (alpha <= bound).
        const float bound = (d_obs - r_clearance_mm) * envelope;
        alpha_max.at(i) = std::min(alpha_max.at(i), bound);
        alpha_min.at(i) = std::min(alpha_min.at(i), bound - obstacle_margin_mm);
      }
    }
  }

  // Pin endpoints to centerline
  alpha_min.front() = 0.0f;
  alpha_max.front() = 0.0f;
  alpha_min.back() = 0.0f;
  alpha_max.back() = 0.0f;
}

struct QPMatrix {
  std::vector<float> h_diag;
  std::vector<float> h_sub1;
  std::vector<float> h_sup1;
  std::vector<float> h_sub2;
  std::vector<float> h_sup2;
  std::vector<float> f;
};

QPMatrix assemble_qp_system(size_t n, const CenterlineGeometry& geom, float curvature_weight,
                            float reg_weight) {
  std::vector<float> a_k(n, 0.0f);
  std::vector<float> b_k(n, 0.0f);

  for (size_t k = 1; k + 1 < n; ++k) {
    const float ds = std::max(0.5f * (geom.ds_m.at(k) + geom.ds_m.at(k + 1)), 0.001f);
    const float c_k = 0.001f / (ds * ds);
    a_k.at(k) = curvature_weight * ds * c_k * c_k;
    b_k.at(k) = curvature_weight * ds * c_k * geom.kappa_cl.at(k);
  }

  QPMatrix qp{std::vector<float>(n, reg_weight), std::vector<float>(n, 0.0f),
              std::vector<float>(n, 0.0f),       std::vector<float>(n, 0.0f),
              std::vector<float>(n, 0.0f),       std::vector<float>(n, 0.0f)};

  for (size_t i = 0; i < n; ++i) {
    if (i >= 2 && i - 1 < n - 1) {
      const size_t k = i - 1;
      qp.h_diag.at(i) += a_k.at(k);
      qp.h_sub1.at(i) += -2.0f * a_k.at(k);
      qp.h_sub2.at(i) += a_k.at(k);
      qp.f.at(i) += b_k.at(k);
    }
    if (i >= 1 && i < n - 1) {
      const size_t k = i;
      qp.h_diag.at(i) += 4.0f * a_k.at(k);
      qp.h_sub1.at(i) += -2.0f * a_k.at(k);
      qp.h_sup1.at(i) += -2.0f * a_k.at(k);
      qp.f.at(i) += -2.0f * b_k.at(k);
    }
    if (i + 1 < n - 1) {
      const size_t k = i + 1;
      qp.h_diag.at(i) += a_k.at(k);
      qp.h_sup1.at(i) += -2.0f * a_k.at(k);
      qp.h_sup2.at(i) += a_k.at(k);
      qp.f.at(i) += b_k.at(k);
    }
  }
  return qp;
}

std::vector<float> solve_projected_gauss_seidel(size_t n, const QPMatrix& qp,
                                                const std::vector<float>& alpha_min,
                                                const std::vector<float>& alpha_max,
                                                int max_iterations, float tolerance_mm) {
  std::vector<float> alpha(n, 0.0f);
  for (size_t i = 0; i < n; ++i) {
    alpha.at(i) = 0.5f * (alpha_min.at(i) + alpha_max.at(i));
  }

  for (int iter = 0; iter < max_iterations; ++iter) {
    float max_diff = 0.0f;
    for (size_t i = 0; i < n; ++i) {
      float sigma = qp.f.at(i);
      if (i >= 2) {
        sigma += qp.h_sub2.at(i) * alpha.at(i - 2);
      }
      if (i >= 1) {
        sigma += qp.h_sub1.at(i) * alpha.at(i - 1);
      }
      if (i + 1 < n) {
        sigma += qp.h_sup1.at(i) * alpha.at(i + 1);
      }
      if (i + 2 < n) {
        sigma += qp.h_sup2.at(i) * alpha.at(i + 2);
      }

      const float unconstrained = -sigma / qp.h_diag.at(i);
      const float clamped = std::clamp(unconstrained, alpha_min.at(i), alpha_max.at(i));

      const float diff = std::abs(clamped - alpha.at(i));
      max_diff = std::max(diff, max_diff);
      alpha.at(i) = clamped;
    }

    if (max_diff < tolerance_mm) {
      break;
    }
  }
  return alpha;
}

std::vector<TrajectoryPoint> build_optimized_trajectory(
  const std::vector<TrajectoryPoint>& centerline, const CenterlineGeometry& geom,
  const std::vector<float>& alpha) {
  const size_t n = centerline.size();
  std::vector<TrajectoryPoint> result = centerline;
  for (size_t i = 0; i < n; ++i) {
    result.at(i).x_mm = centerline.at(i).x_mm + alpha.at(i) * geom.nx.at(i);
    result.at(i).y_mm = centerline.at(i).y_mm + alpha.at(i) * geom.ny.at(i);
  }

  float total_s = 0.0f;
  result.front().s_m = 0.0f;
  for (size_t i = 1; i < n; ++i) {
    total_s += std::hypot(result.at(i).x_mm - result.at(i - 1).x_mm,
                          result.at(i).y_mm - result.at(i - 1).y_mm) *
               0.001f;
    result.at(i).s_m = total_s;
  }

  for (size_t i = 0; i < n; ++i) {
    const size_t prev_i = (i > 0) ? (i - 1) : 0;
    const size_t next_i = (i + 1 < n) ? (i + 1) : (n - 1);
    const float dx = result.at(next_i).x_mm - result.at(prev_i).x_mm;
    const float dy = result.at(next_i).y_mm - result.at(prev_i).y_mm;
    result.at(i).heading_rad = std::atan2(dy, dx);
  }

  result.front().curvature_rad_m = 0.0f;
  result.back().curvature_rad_m = 0.0f;
  for (size_t i = 1; i + 1 < n; ++i) {
    result.at(i).curvature_rad_m =
      menger_curvature_rad_m(result.at(i - 1).x_mm, result.at(i - 1).y_mm, result.at(i).x_mm,
                             result.at(i).y_mm, result.at(i + 1).x_mm, result.at(i + 1).y_mm, true);
  }

  return result;
}

}  // namespace

GlobalTrajectoryOptimizer::GlobalTrajectoryOptimizer() : config_(OptimizationConfig{}) {}

GlobalTrajectoryOptimizer::GlobalTrajectoryOptimizer(const OptimizationConfig& config)
  : config_(config) {}

std::vector<TrajectoryPoint> GlobalTrajectoryOptimizer::optimize(
  const std::vector<TrajectoryPoint>& centerline, const std::vector<ObstacleGeometry>& obstacles,
  const EnvironmentConfig& env) const {
  const size_t n = centerline.size();
  if (n < 3) {
    return centerline;
  }

  const CenterlineGeometry geom = compute_centerline_geometry(centerline);

  std::vector<float> alpha_min(n, -config_.max_lateral_deviation_mm);
  std::vector<float> alpha_max(n, config_.max_lateral_deviation_mm);
  apply_obstacle_corridor_constraints(centerline, obstacles, env, geom, config_.obstacle_margin_mm,
                                      alpha_min, alpha_max);

  const QPMatrix qp = assemble_qp_system(n, geom, config_.curvature_weight, config_.reg_weight);

  const std::vector<float> alpha = solve_projected_gauss_seidel(
    n, qp, alpha_min, alpha_max, config_.max_iterations, config_.tolerance_mm);

  return build_optimized_trajectory(centerline, geom, alpha);
}

}  // namespace lf
