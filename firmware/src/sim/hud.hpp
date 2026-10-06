#pragma once

#include <cstddef>
#include <vector>

#include "config/types.hpp"
#include "raylib.h"
#include "system/robot_coordinator.hpp"
#include "track/trajectory_planner.hpp"
#include "types.hpp"

namespace lf::sim {

enum class GuiAction : uint8_t {
  kNone = 0,
  kAutoTune,
  kRun1Map,
  kRun2Sprint,
  kTogglePause,
  kResetPose,
  kResetAll,
  kToggleFollowCam,
  kFitArena,
  kClearObstacles,
  kCommitC2,
};

struct HudResult {
  GuiAction action = GuiAction::kNone;
  bool tactical_changed = false;
};

// Hallmark Cockpit Design Tokens
struct HudTheme {
  static constexpr Color kBg = {11, 15, 23, 255};              // #0B0F17 Cockpit dark
  static constexpr Color kSurface = {17, 24, 39, 255};         // #111827 Flat panel
  static constexpr Color kSurfaceLight = {24, 33, 53, 255};    // #182135 Sub-element
  static constexpr Color kBorder = {30, 41, 59, 255};          // #1E293B 1px hairline
  static constexpr Color kBorderLight = {45, 60, 85, 255};     // #2D3C55 Subtle divider
  static constexpr Color kBorderActive = {56, 189, 248, 255};  // #38BDF8 Focus
  static constexpr Color kTextMain = {248, 250, 252, 255};     // #F8FAFC High-contrast
  static constexpr Color kTextMuted = {148, 163, 184, 255};    // #94A3B8 Secondary
  static constexpr Color kTextDim = {100, 116, 139, 255};      // #64748B Tertiary
  static constexpr Color kAccent = {56, 189, 248, 255};        // #38BDF8 High-tech Cyan
  static constexpr Color kSuccess = {16, 185, 129, 255};       // #10B981 Emerald
  static constexpr Color kWarn = {245, 158, 11, 255};          // #F59E0B Amber
  static constexpr Color kAlert = {239, 68, 68, 255};          // #EF4444 Red
};

void draw_text_crisp(const Font& font, const char* text, float x, float y, float font_size,
                     Color color);

// Primary entry point for sidebar telemetry rendering
HudResult draw_hud_sidebar(float sb_x, float sb_w, float view_h, float ui_scale,
                           lf::RobotCoordinator& robot, const lf::SensorData& in,
                           SimRobotState& state, size_t obstacle_count,
                           float hovered_obstacle_angle, Vector2 mouse_pos, bool mouse_down,
                           bool mouse_pressed, bool follow_robot, const char* toast_msg,
                           float toast_timer, const Font& font_bold, const Font& font_mono);

// Canvas overlay hover tooltip
void draw_canvas_hover_telemetry(Vector2 mouse_screen, const lf::TrajectoryPoint& pt, size_t pt_idx,
                                 float ui_scale, const Font& font_bold, const Font& font_mono);

}  // namespace lf::sim
