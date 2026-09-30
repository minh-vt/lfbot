#include "raycast.hpp"

#include <algorithm>
#include <cmath>

namespace lf::sim {

namespace {

float raycast_boundary(Vector2 origin, float dir_x, float dir_y, float current_min_mm) {
  float min_d = current_min_mm;
  if (dir_x > 1e-5f) {
    const float d = (kTrackWidthMm - origin.x) / dir_x;
    if (d > 0.0f && d < min_d) {
      min_d = d;
    }
  } else if (dir_x < -1e-5f) {
    const float d = (0.0f - origin.x) / dir_x;
    if (d > 0.0f && d < min_d) {
      min_d = d;
    }
  }

  if (dir_y > 1e-5f) {
    const float d = (kTrackHeightMm - origin.y) / dir_y;
    if (d > 0.0f && d < min_d) {
      min_d = d;
    }
  } else if (dir_y < -1e-5f) {
    const float d = (0.0f - origin.y) / dir_y;
    if (d > 0.0f && d < min_d) {
      min_d = d;
    }
  }
  return min_d;
}

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

float raycast_obstacle_distance(Vector2 origin, float angle_rad, float max_dist_mm,
                                const std::vector<Obstacle>& obstacles, Vector2* out_hit) {
  const float dir_x = std::cos(angle_rad);
  const float dir_y = std::sin(angle_rad);

  float min_dist_mm = max_dist_mm;

  for (const auto& obs : obstacles) {
    min_dist_mm = raycast_obb(origin, dir_x, dir_y, min_dist_mm, obs);
  }

  // Tinh vi tri cat tia de ve visual tren man hinh
  float visual_dist_mm = min_dist_mm;
  if (visual_dist_mm >= max_dist_mm) {
    visual_dist_mm = raycast_boundary(origin, dir_x, dir_y, max_dist_mm);
  }

  if (out_hit != nullptr) {
    out_hit->x = origin.x + dir_x * visual_dist_mm;
    out_hit->y = origin.y + dir_y * visual_dist_mm;
  }

  return min_dist_mm * 0.001f;
}

void raycast_all_tof(float robot_x, float robot_y, float robot_theta, float bar_center_x,
                     float bar_center_y, const std::vector<Obstacle>& obstacles, lf::SensorData& in,
                     TofHitPoints& hits, Vector2& out_l_origin, Vector2& out_r_origin) {
  const float cos_th = std::cos(robot_theta);
  const float sin_th = std::sin(robot_theta);

  in.tof_front_m = raycast_obstacle_distance(Vector2{bar_center_x, bar_center_y}, robot_theta,
                                             1200.0f, obstacles, &hits.front);

  out_l_origin = Vector2{robot_x - (kRobotWidthMm * 0.45f) * (-sin_th),
                         robot_y - (kRobotWidthMm * 0.45f) * cos_th};
  in.tof_left_m = raycast_obstacle_distance(out_l_origin, robot_theta - 1.5707963f, 600.0f,
                                            obstacles, &hits.left);

  out_r_origin = Vector2{robot_x + (kRobotWidthMm * 0.45f) * (-sin_th),
                         robot_y + (kRobotWidthMm * 0.45f) * cos_th};
  in.tof_right_m = raycast_obstacle_distance(out_r_origin, robot_theta + 1.5707963f, 600.0f,
                                             obstacles, &hits.right);

  in.tof_side_m = std::min(in.tof_left_m, in.tof_right_m);
}

}  // namespace lf::sim
