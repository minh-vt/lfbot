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

constexpr float kTrackWidthMm = 2400.0f;
constexpr float kTrackHeightMm = 1600.0f;

constexpr float kRobotLengthMm = 145.0f;
constexpr float kRobotWidthMm = 138.0f;
constexpr float kWheelLengthMm = 44.0f;
constexpr float kWheelWidthMm = 14.0f;

constexpr float kStartPosX = 500.0f;
constexpr float kStartPosY = 1300.0f;

constexpr size_t kMaxTrailPoints = 4000;

struct Obstacle {
  Vector2 pos{};
  float width_mm = 105.0f;
  float height_mm = 60.0f;
  float angle_deg = 0.0f;
};

struct TofHitPoints {
  Vector2 front{};
  Vector2 left{};
  Vector2 right{};
};

struct SimRobotState {
  float x = kStartPosX;
  float y = kStartPosY;
  float theta = 0.0f;
  float v = 0.0f;
  float w = 0.0f;
  float left_pwm = 0.0f;
  float right_pwm = 0.0f;
  int32_t enc_l = 0;
  int32_t enc_r = 0;
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
};

}  // namespace lf::sim
