#pragma once

#include <cstdint>
#include <deque>
#include <vector>

#include "config/types.hpp"
#include "raylib.h"

namespace lf::sim {

constexpr int kBaseWindowWidth = 1920;
constexpr int kBaseWindowHeight = 1080;
constexpr float kBaseSidebarWidth = 490.0f;

constexpr float kRobotLengthMm = 145.0f;
constexpr float kRobotWidthMm = 138.0f;
constexpr float kWheelLengthMm = 44.0f;
constexpr float kWheelWidthMm = 14.0f;

constexpr size_t kMaxTrailPoints = 4000;

struct Obstacle {
  Vector2 pos{};
  float width_mm = 105.0f;
  float height_mm = 60.0f;
  float angle_deg = 0.0f;
};

struct TofHitPoint {
  float x = 0.0f;
  float y = 0.0f;
  float distance_mm = 0.0f;
  bool has_hit = false;

  [[nodiscard]] Vector2 to_vec2() const { return Vector2{x, y}; }
};

struct TofHitPoints {
  TofHitPoint front{};
  TofHitPoint left{};
  TofHitPoint right{};
};

enum class SetupTool : uint8_t {
  kSelect = 0,
  kSetStart,
  kSetFinish,
  kAddObstacle,
};

struct SimRobotState {
  float x = 0.0f;
  float y = 0.0f;
  float theta = 0.0f;
  float v = 0.0f;
  float w = 0.0f;
  float left_pwm = 0.0f;
  float right_pwm = 0.0f;
  int32_t enc_l = 0;
  int32_t enc_r = 0;
  double wheel_dist_l_m = 0.0;
  double wheel_dist_r_m = 0.0;
  bool is_paused = true;
  float sim_time = 0.0f;
  std::deque<Vector2> trail;

  // Du lieu so sanh quy dao giua Run 1 va Run 2
  std::vector<Vector2> unoptimized_trail;
  std::vector<lf::TrajectoryPoint> planned_trajectory;
  float run1_time_s = 0.0f;
  float run2_time_s = 0.0f;
  bool has_run1_data = false;
  bool has_run2_data = false;
  int hovered_obstacle_idx = -1;
  bool is_telemetry_mode = false;
  uint32_t packets_received = 0;
  bool is_auto_scale_cam = false;
  int focused_param_idx = 0;
  int last_pressed_key = 0;
  float key_flash_timer = 0.0f;
  float lateral_accel_mps2 = 0.0f;
  float longitudinal_accel_mps2 = 0.0f;
  float prev_v = 0.0f;
  int hovered_trajectory_idx = -1;
  bool is_crashed = false;
  int crashed_obstacle_idx = -1;
  bool is_setup_mode = false;
  SetupTool setup_tool = SetupTool::kSelect;
};

}  // namespace lf::sim
