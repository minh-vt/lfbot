#pragma once

#include <raylib.h>

#include <cstddef>
#include <cstdint>
#include <string>

#include "system/robot_coordinator.hpp"
#include "theme.hpp"
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
  kPrevTrack,
  kNextTrack,
  kSaveTrack,
  kToggleSetupMode,
  kSetToolSelect,
  kSetToolStart,
  kSetToolFinish,
  kSetToolAddObs,
  kDeleteHoveredObs,
  kToggleTuningDrawer,
  kTuneKpUp,
  kTuneKpDown,
  kTuneKdUp,
  kTuneKdDown,
  kTuneVmapUp,
  kTuneVmapDown,
  kTuneVsprintUp,
  kTuneVsprintDown,
  kTuneAccelUp,
  kTuneAccelDown,
  kTuneObsClearUp,
  kTuneObsClearDown,
  kTuneJerkUp,
  kTuneJerkDown,
  kTuneCorridorUp,
  kTuneCorridorDown,
  kTriggerLiveReopt,
};

struct HudResult {
  GuiAction action = GuiAction::kNone;
  bool tactical_changed = false;
};

struct ButtonSpec {
  const char* label = nullptr;
  GuiAction action = GuiAction::kNone;
  Color accent = Theme::kAccent;
  bool is_active = false;
};

struct TelemetryCard {
  const char* label = nullptr;
  std::string value;
  Color accent = Theme::kTextMain;
};

void draw_text_crisp(const Font& font, const char* text, float x, float y, float font_size,
                     Color color);

// Primary Hallmark Tactical HUD:
// Renders top floating status/control pill and bottom monospace telemetry ribbon
HudResult draw_tactical_hud(float screen_w, float screen_h, float ui_scale,
                            const lf::RobotCoordinator& robot, const lf::SensorData& in,
                            SimRobotState& state, size_t obstacle_count,
                            float hovered_obstacle_angle, bool follow_robot, const char* toast_msg,
                            float toast_timer, const Font& font_bold, const Font& font_mono,
                            const char* track_name = nullptr);

// Canvas overlay hover tooltip
void draw_canvas_hover_telemetry(Vector2 mouse_screen, const lf::TrajectoryPoint& pt, size_t pt_idx,
                                 float ui_scale, const Font& font_bold, const Font& font_mono);

}  // namespace lf::sim
