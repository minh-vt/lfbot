#pragma once

#include <array>
#include <deque>
#include <vector>

#include "config/robot_config.hpp"
#include "config/types.hpp"
#include "raylib.h"
#include "types.hpp"

namespace lf::sim {

void draw_rotated_box(Vector2 center, float width, float height, float angle_deg, Color fill,
                      Color border = BLANK, float border_thick = 0.0f);

void draw_hazard_obstacle(const Obstacle& obs, bool is_hovered = false,
                          float clearance_radius_mm = 155.0f);

void draw_arena_background(Texture2D track_texture);

void draw_robot_trail(const std::deque<Vector2>& trail, lf::RobotMode mode);

void draw_trajectory_comparison(const std::vector<Vector2>& unoptimized_trail,
                                const std::vector<lf::TrajectoryPoint>& planned_trajectory,
                                float max_speed_mps);

void draw_robot_chassis(float robot_x, float robot_y, float robot_theta, float bar_center_x,
                        float bar_center_y, float wheel_base_m, const lf::SensorData& in,
                        const lf::Telemetry& tel,
                        const std::array<float, lf::LineSensorConfig::kSensorCount>& weights_mm,
                        bool is_crashed = false);

void draw_obstacles(const std::vector<Obstacle>& obstacles, int hovered_idx = -1,
                    float clearance_radius_mm = 155.0f, int crashed_idx = -1);

void draw_tof_beams(float bar_center_x, float bar_center_y, Vector2 l_origin, Vector2 r_origin,
                    float robot_theta, const TofHitPoints& hits, const lf::SensorData& in,
                    float d_obs_m = 0.35f);

}  // namespace lf::sim
