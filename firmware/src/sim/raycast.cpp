#include "raycast.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace lf::sim {

namespace {

constexpr float kToFFoVRad = 25.0f * DEG2RAD;  // 25 degree Field-of-View cone

float raycast_obb(Vector2 origin, float dir_x, float dir_y, float current_min_mm,
                  const Obstacle& obs) {
  const float obs_rad = obs.angle_deg * DEG2RAD;
  const float cos_obs = std::cos(-obs_rad);
  const float sin_obs = std::sin(-obs_rad);

  const float dx = origin.x - obs.pos.x;
  const float dy = origin.y - obs.pos.y;
  const float local_ox = dx * cos_obs - dy * sin_obs;
  const float local_oy = dx * sin_obs + dy * cos_obs;

  const float local_dx = dir_x * cos_obs - dir_y * sin_obs;
  const float local_dy = dir_x * sin_obs + dir_y * cos_obs;

  const float hw = obs.width_mm * 0.5f;
  const float hh = obs.height_mm * 0.5f;

  float tmin = 0.0f;
  float tmax = current_min_mm;

  if (std::abs(local_dx) > 1e-5f) {
    float tx1 = (-hw - local_ox) / local_dx;
    float tx2 = (hw - local_ox) / local_dx;
    if (tx1 > tx2) {
      std::swap(tx1, tx2);
    }
    tmin = std::max(tmin, tx1);
    tmax = std::min(tmax, tx2);
    if (tmin > tmax) {
      return current_min_mm;
    }
  } else if (local_ox < -hw || local_ox > hw) {
    return current_min_mm;
  }

  if (std::abs(local_dy) > 1e-5f) {
    float ty1 = (-hh - local_oy) / local_dy;
    float ty2 = (hh - local_oy) / local_dy;
    if (ty1 > ty2) {
      std::swap(ty1, ty2);
    }
    tmin = std::max(tmin, ty1);
    tmax = std::min(tmax, ty2);
    if (tmin > tmax) {
      return current_min_mm;
    }
  } else if (local_oy < -hh || local_oy > hh) {
    return current_min_mm;
  }

  if (tmin > 0.0f && tmin < current_min_mm) {
    return tmin;
  }
  return current_min_mm;
}

}  // namespace

float raycast_obstacle_distance(Vector2 origin, float angle_rad, float fov_rad, float max_dist_mm,
                                const std::vector<Obstacle>& obstacles, TofHitPoint* out_hit) {
  // Dense conical sector casting: 21 rays spanning the FoV cone [-fov/2, +fov/2]
  constexpr int kRayCount = 21;
  const float half_fov = fov_rad * 0.5f;
  const float angle_step = fov_rad / static_cast<float>(kRayCount - 1);

  float overall_min_mm = max_dist_mm;
  float best_dir_x = std::cos(angle_rad);
  float best_dir_y = std::sin(angle_rad);
  bool has_hit = false;

  for (int i = 0; i < kRayCount; ++i) {
    const float ray_angle = (angle_rad - half_fov) + static_cast<float>(i) * angle_step;
    const float dir_x = std::cos(ray_angle);
    const float dir_y = std::sin(ray_angle);

    float ray_min_mm = max_dist_mm;
    for (const auto& obs : obstacles) {
      ray_min_mm = raycast_obb(origin, dir_x, dir_y, ray_min_mm, obs);
    }

    if (ray_min_mm < overall_min_mm) {
      overall_min_mm = ray_min_mm;
      best_dir_x = dir_x;
      best_dir_y = dir_y;
      has_hit = true;
    }
  }

  if (out_hit != nullptr) {
    out_hit->has_hit = has_hit;
    out_hit->distance_mm = overall_min_mm;
    if (has_hit) {
      out_hit->x = origin.x + best_dir_x * overall_min_mm;
      out_hit->y = origin.y + best_dir_y * overall_min_mm;
    } else {
      out_hit->x = origin.x + std::cos(angle_rad) * max_dist_mm;
      out_hit->y = origin.y + std::sin(angle_rad) * max_dist_mm;
    }
  }

  if (has_hit) {
    return overall_min_mm * 0.001f;
  }
  return max_dist_mm * 0.001f;
}

void raycast_all_tof(float robot_x, float robot_y, float robot_theta, float bar_center_x,
                     float bar_center_y, const std::vector<Obstacle>& obstacles, lf::SensorData& in,
                     TofHitPoints& hits, Vector2& out_l_origin, Vector2& out_r_origin,
                     float max_front_dist_m, float max_side_dist_m) {
  const float cos_th = std::cos(robot_theta);
  const float sin_th = std::sin(robot_theta);

  const float front_max_mm = std::max(50.0f, max_front_dist_m * 1000.0f);
  const float side_max_mm = std::max(50.0f, max_side_dist_m * 1000.0f);

  in.tof_front_m = raycast_obstacle_distance(Vector2{bar_center_x, bar_center_y}, robot_theta,
                                             kToFFoVRad, front_max_mm, obstacles, &hits.front);

  out_l_origin = Vector2{robot_x - (kRobotWidthMm * 0.45f) * (-sin_th),
                         robot_y - (kRobotWidthMm * 0.45f) * cos_th};
  in.tof_left_m = raycast_obstacle_distance(out_l_origin, robot_theta - 1.5707963f, kToFFoVRad,
                                            side_max_mm, obstacles, &hits.left);

  out_r_origin = Vector2{robot_x + (kRobotWidthMm * 0.45f) * (-sin_th),
                         robot_y + (kRobotWidthMm * 0.45f) * cos_th};
  in.tof_right_m = raycast_obstacle_distance(out_r_origin, robot_theta + 1.5707963f, kToFFoVRad,
                                             side_max_mm, obstacles, &hits.right);
}

}  // namespace lf::sim
