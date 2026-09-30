#pragma once

#include <vector>

#include "config/types.hpp"
#include "raylib.h"
#include "types.hpp"

namespace lf::sim {

float raycast_obstacle_distance(Vector2 origin, float angle_rad, float max_dist_mm,
                                const std::vector<Obstacle>& obstacles, Vector2* out_hit = nullptr);

void raycast_all_tof(float robot_x, float robot_y, float robot_theta, float bar_center_x,
                     float bar_center_y, const std::vector<Obstacle>& obstacles, lf::SensorData& in,
                     TofHitPoints& hits, Vector2& out_l_origin, Vector2& out_r_origin);

}  // namespace lf::sim
