#pragma once

#include <vector>

#include "config/types.hpp"
#include "raylib.h"
#include "types.hpp"

namespace lf::sim {

float raycast_obstacle_distance(Vector2 origin, float angle_rad, float fov_rad, float max_dist_mm,
                                const std::vector<Obstacle>& obstacles,
                                TofHitPoint* out_hit = nullptr);

void raycast_all_tof(float robot_x, float robot_y, float robot_theta, float bar_center_x,
                     float bar_center_y, const std::vector<Obstacle>& obstacles, lf::SensorData& in,
                     TofHitPoints& hits, Vector2& out_l_origin, Vector2& out_r_origin,
                     float max_front_dist_m = 0.35f, float max_side_dist_m = 0.20f);

}  // namespace lf::sim
