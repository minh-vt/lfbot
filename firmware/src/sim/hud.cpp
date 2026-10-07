#include "hud.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "config/constants.hpp"

namespace lf::sim {

namespace {

constexpr float kWheelDiameterM = 0.034f;
constexpr float kCircumferenceM = constants::kPi * kWheelDiameterM;  // 0.106814 m

Vector2 measure_text_dim(const Font& font, const char* text, float font_size) {
  if (text == nullptr || text[0] == '\0') {
    return Vector2{0.0f, 0.0f};
  }
  if (font.texture.id > 0) {
    return MeasureTextEx(font, text, font_size, 1.0f);
  }
  return Vector2{static_cast<float>(MeasureText(text, static_cast<int>(font_size))), font_size};
}

bool draw_compact_btn(Rectangle rect, const char* label, Color accent, bool is_active,
                      const Font& font_bold, float font_sz, GuiAction act, GuiAction& out_act) {
  const Vector2 mouse_pos = GetMousePosition();
  const bool is_hovered = CheckCollisionPointRec(mouse_pos, rect);
  const bool is_clicked = is_hovered && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);

  Color bg = Theme::kSurface;
  Color border = Theme::kBorder;
  Color text_col = accent;

  if (is_active) {
    bg = ColorAlpha(accent, 0.28f);
    border = accent;
    text_col = Theme::kTextMain;
  } else if (is_hovered) {
    bg = ColorAlpha(accent, 0.16f);
    border = accent;
    text_col = Theme::kTextMain;
  }

  DrawRectangleRounded(rect, 0.32f, 4, bg);
  DrawRectangleRoundedLines(rect, 0.32f, 4, border);

  const Vector2 text_sz = measure_text_dim(font_bold, label, font_sz);
  const float text_x =
    (font_sz > 0 && text_sz.x > 0) ? (rect.x + (rect.width - text_sz.x) * 0.5f) : (rect.x + 8.0f);
  const float text_y = rect.y + (rect.height - font_sz) * 0.5f;

  draw_text_crisp(font_bold, label, text_x, text_y, font_sz, text_col);

  if (is_clicked) {
    out_act = act;
    return true;
  }
  return false;
}

// Reusable ButtonSpec renderer with dynamic width measurement
float draw_spec_button(float cur_x, float btn_y, float btn_h, float font_sz, float ui_scale,
                       const ButtonSpec& spec, const Font& font_bold, GuiAction& out_act) {
  const Vector2 label_sz = measure_text_dim(font_bold, spec.label, font_sz);
  const float btn_w = label_sz.x + 2.0f * Theme::kButtonPaddingX * ui_scale;
  draw_compact_btn(Rectangle{cur_x, btn_y, btn_w, btn_h}, spec.label, spec.accent, spec.is_active,
                   font_bold, font_sz, spec.action, out_act);
  return btn_w;
}

// Reusable vertical divider line
void draw_hairline_divider(float x, float y, float h) {
  DrawLine(static_cast<int>(x), static_cast<int>(y), static_cast<int>(x), static_cast<int>(y + h),
           Theme::kBorder);
}

// Reusable professional telemetry card component
float draw_telemetry_card_component(float cur_x, float rib_y, float rib_h, float ui_scale,
                                    const TelemetryCard& card, const Font& font_bold,
                                    const Font& font_mono) {
  (void)rib_h;
  const float font_lbl_sz = Theme::kFontSmall * ui_scale;
  const float font_val_sz = Theme::kFontLabel * ui_scale;

  const Vector2 lbl_sz = measure_text_dim(font_bold, card.label, font_lbl_sz);
  const Vector2 val_sz = measure_text_dim(font_mono, card.value.c_str(), font_val_sz);

  const float card_w = std::max(lbl_sz.x, val_sz.x) + 2.0f * Theme::kCardPaddingX * ui_scale;

  // Vertical layout: Micro-caption on top, high-contrast metric below
  const float lbl_y = rib_y + 4.0f * ui_scale;
  const float val_y = rib_y + 19.0f * ui_scale;

  draw_text_crisp(font_bold, card.label, cur_x + Theme::kCardPaddingX * ui_scale, lbl_y,
                  font_lbl_sz, Theme::kTextDim);
  draw_text_crisp(font_mono, card.value.c_str(), cur_x + Theme::kCardPaddingX * ui_scale, val_y,
                  font_val_sz, card.accent);

  return card_w;
}

void draw_top_control_pill(float screen_w, float ui_scale, const lf::RobotCoordinator& robot,
                           const SimRobotState& state, const char* track_name,
                           const Font& font_bold, const Font& font_mono, GuiAction& out_act) {
  const float pill_h = Theme::kTopPillHeight * ui_scale;
  const float max_pill_w = 1360.0f * ui_scale;
  const float pill_w = std::min(screen_w - 32.0f * ui_scale, max_pill_w);
  const float pill_x = (screen_w - pill_w) * 0.5f;
  const float pill_y = 12.0f * ui_scale;
  const Rectangle pill_rect{pill_x, pill_y, pill_w, pill_h};

  DrawRectangleRounded(pill_rect, 0.35f, 6, ColorAlpha(Theme::kSurface, 0.94f));
  DrawRectangleRoundedLines(pill_rect, 0.35f, 6, Theme::kBorder);

  float cur_x = pill_x + 10.0f * ui_scale;
  const float btn_h = 26.0f * ui_scale;
  const float btn_y = pill_y + (pill_h - btn_h) * 0.5f;
  const float font_sz = Theme::kFontButton * ui_scale;

  // 1. Track Switcher Pill [< Track Name >]
  const float arrow_w = 24.0f * ui_scale;
  draw_compact_btn(Rectangle{cur_x, btn_y, arrow_w, btn_h}, "<", Theme::kTextDim, false, font_bold,
                   font_sz, GuiAction::kPrevTrack, out_act);
  cur_x += arrow_w + 3.0f * ui_scale;

  std::array<char, 64> track_buf{};
  (void)snprintf(track_buf.data(), track_buf.size(), "%s",
                 (track_name != nullptr && track_name[0] != '\0') ? track_name : "Track");
  const Vector2 track_label_sz = measure_text_dim(font_bold, track_buf.data(), font_sz);
  const float track_name_w = track_label_sz.x + 24.0f * ui_scale;
  draw_compact_btn(Rectangle{cur_x, btn_y, track_name_w, btn_h}, track_buf.data(), Theme::kAccent,
                   false, font_bold, font_sz, GuiAction::kNextTrack, out_act);
  cur_x += track_name_w + 3.0f * ui_scale;

  draw_compact_btn(Rectangle{cur_x, btn_y, arrow_w, btn_h}, ">", Theme::kTextDim, false, font_bold,
                   font_sz, GuiAction::kNextTrack, out_act);
  cur_x += arrow_w + 10.0f * ui_scale;

  draw_hairline_divider(cur_x, pill_y + 8.0f * ui_scale, pill_h - 16.0f * ui_scale);
  cur_x += 10.0f * ui_scale;

  // 2. Status Badge
  const auto mode = robot.get_mode();
  const char* badge_text = "IDLE";
  Color badge_col = Theme::kTextDim;
  if (state.is_crashed) {
    badge_text = "CRASH";
    badge_col = Theme::kAlert;
  } else if (state.is_setup_mode) {
    badge_text = "SETUP";
    badge_col = Theme::kSuccess;
  } else if (state.is_paused) {
    badge_text = "PAUSED";
    badge_col = Theme::kWarn;
  } else if (mode == lf::RobotMode::kAutoTune) {
    badge_text = "TUNE";
    badge_col = Theme::kSuccess;
  } else if (mode == lf::RobotMode::kRun1Map) {
    badge_text = "RUN 1: MAP";
    badge_col = Theme::kAccent;
  } else if (mode == lf::RobotMode::kRun2Sprint) {
    badge_text = "RUN 2: SPRINT";
    badge_col = Theme::kWarn;
  }

  const Vector2 badge_sz = measure_text_dim(font_bold, badge_text, font_sz);
  const float badge_w = badge_sz.x + 20.0f * ui_scale;
  DrawRectangleRounded(Rectangle{cur_x, btn_y, badge_w, btn_h}, 0.30f, 4,
                       ColorAlpha(badge_col, 0.16f));
  DrawRectangleRoundedLines(Rectangle{cur_x, btn_y, badge_w, btn_h}, 0.30f, 4, badge_col);
  draw_text_crisp(font_bold, badge_text, cur_x + 10.0f * ui_scale, btn_y + 6.0f * ui_scale, font_sz,
                  badge_col);
  cur_x += badge_w + 10.0f * ui_scale;

  draw_hairline_divider(cur_x, pill_y + 8.0f * ui_scale, pill_h - 16.0f * ui_scale);
  cur_x += 10.0f * ui_scale;

  // 3. Lap Times / Telemetry Clock
  std::array<char, 64> lap_buf{};
  if (state.has_run2_data) {
    const float delta = state.run2_time_s - state.run1_time_s;
    const float pct = (state.run1_time_s > 0.01f) ? (delta / state.run1_time_s * 100.0f) : 0.0f;
    (void)snprintf(lap_buf.data(), lap_buf.size(), "R1:%04.1fs  R2:%04.1fs (%+.0f%%)",
                   state.run1_time_s, state.run2_time_s, pct);
  } else if (state.has_run1_data) {
    (void)snprintf(lap_buf.data(), lap_buf.size(), "R1:%04.1fs  T:%04.1fs", state.run1_time_s,
                   state.sim_time);
  } else {
    (void)snprintf(lap_buf.data(), lap_buf.size(), "T:%05.2fs", state.sim_time);
  }
  draw_text_crisp(font_mono, lap_buf.data(), cur_x, pill_y + 12.0f * ui_scale, 14.0f * ui_scale,
                  state.is_paused ? Theme::kWarn : Theme::kTextMain);

  // 4. Quick Actions on the Right (Rendered dynamically using ButtonSpec array)
  float right_x = pill_x + pill_w - 8.0f * ui_scale;
  const std::array<ButtonSpec, 6> right_buttons = {
    {
      {
        state.is_setup_mode ? "[E] EDITING" : "[E] SETUP",
        GuiAction::kToggleSetupMode,
        Theme::kSuccess,
        state.is_setup_mode,
      },
      {"[F] FIT", GuiAction::kFitArena, Theme::kTextDim, false},
      {"[R] RESET", GuiAction::kResetPose, Theme::kAlert, false},
      {
        state.is_paused ? "[>] RUN" : "[||] PAUSE",
        GuiAction::kTogglePause,
        state.is_paused ? Theme::kSuccess : Theme::kTextMain,
        false,
      },
      {"[2] SPRINT", GuiAction::kRun2Sprint, Theme::kWarn, mode == lf::RobotMode::kRun2Sprint},
      {"[1] MAP", GuiAction::kRun1Map, Theme::kAccent, mode == lf::RobotMode::kRun1Map},
    },
  };

  for (const auto& spec : right_buttons) {
    const Vector2 sz = measure_text_dim(font_bold, spec.label, font_sz);
    const float btn_w = sz.x + 2.0f * Theme::kButtonPaddingX * ui_scale;
    right_x -= btn_w;
    draw_compact_btn(Rectangle{right_x, btn_y, btn_w, btn_h}, spec.label, spec.accent,
                     spec.is_active, font_bold, font_sz, spec.action, out_act);
    right_x -= 6.0f * ui_scale;
  }
}

void draw_setup_overlay_toolbar(float screen_w, float ui_scale, SimRobotState& state,
                                const Font& font_bold, GuiAction& out_act) {
  const float bar_h = 34.0f * ui_scale;
  const float bar_w = 680.0f * ui_scale;
  const float bar_x = (screen_w - bar_w) * 0.5f;
  const float bar_y = 56.0f * ui_scale;
  const Rectangle bar_rect{bar_x, bar_y, bar_w, bar_h};

  DrawRectangleRounded(bar_rect, 0.35f, 4, ColorAlpha(Theme::kSurfaceLight, 0.95f));
  DrawRectangleRoundedLines(bar_rect, 0.35f, 4, Theme::kBorderActive);

  float cur_x = bar_x + 8.0f * ui_scale;
  const float btn_h = 24.0f * ui_scale;
  const float btn_y = bar_y + (bar_h - btn_h) * 0.5f;
  const float font_sz = Theme::kFontSmall * ui_scale;

  struct ToolSpec {
    const char* label;
    SetupTool tool;
    GuiAction action;
  };

  const std::array<ToolSpec, 5> tools = {
    {
      {"SELECT / DRAG", SetupTool::kSelect, GuiAction::kSetToolSelect},
      {"SET START [S]", SetupTool::kSetStart, GuiAction::kSetToolStart},
      {"SET FINISH [F]", SetupTool::kSetFinish, GuiAction::kSetToolFinish},
      {"+ OBS [O]", SetupTool::kAddObstacle, GuiAction::kSetToolAddObs},
      {"- DEL [DEL]", SetupTool::kSelect, GuiAction::kDeleteHoveredObs},
    },
  };

  for (const auto& t : tools) {
    const bool is_active = (state.setup_tool == t.tool);
    const ButtonSpec spec{
      t.label,
      t.action,
      is_active ? Theme::kSuccess : Theme::kTextDim,
      is_active,
    };
    const float w =
      draw_spec_button(cur_x, btn_y, btn_h, font_sz, ui_scale, spec, font_bold, out_act);
    if (out_act == t.action) {
      state.setup_tool = t.tool;
    }
    cur_x += w + 6.0f * ui_scale;
  }

  // Save manifest button
  const ButtonSpec save_spec{"SAVE JSON [^S]", GuiAction::kSaveTrack, Theme::kSuccess, false};
  draw_spec_button(cur_x, btn_y, btn_h, font_sz, ui_scale, save_spec, font_bold, out_act);
}

void draw_bottom_telemetry_ribbon(float screen_w, float screen_h, float ui_scale,
                                  const lf::RobotCoordinator& robot, const lf::SensorData& in,
                                  const SimRobotState& state, const Font& font_bold,
                                  const Font& font_mono) {
  const float rib_h = Theme::kRibbonHeight * ui_scale;
  const float max_rib_w = 1360.0f * ui_scale;
  const float rib_w = std::min(screen_w - 32.0f * ui_scale, max_rib_w);
  const float rib_x = (screen_w - rib_w) * 0.5f;
  const float rib_y = screen_h - rib_h - 12.0f * ui_scale;
  const Rectangle rib_rect{rib_x, rib_y, rib_w, rib_h};

  DrawRectangleRounded(rib_rect, 0.35f, 6, ColorAlpha(Theme::kBg, 0.94f));
  DrawRectangleRoundedLines(rib_rect, 0.35f, 6, Theme::kBorder);

  float cur_x = rib_x + 8.0f * ui_scale;

  // 1. Speed & Wheel RPM Card
  const int wheel_rpm = static_cast<int>(std::round((state.v * 60.0f) / kCircumferenceM));
  std::array<char, 32> spd_buf{};
  (void)snprintf(spd_buf.data(), spd_buf.size(), "%.2f m/s (%d rpm)", state.v, wheel_rpm);
  cur_x += draw_telemetry_card_component(cur_x, rib_y, rib_h, ui_scale,
                                         TelemetryCard{"SPEED", spd_buf.data(), Theme::kTextMain},
                                         font_bold, font_mono);
  draw_hairline_divider(cur_x, rib_y + 6.0f * ui_scale, rib_h - 12.0f * ui_scale);
  cur_x += Theme::kCardGap * ui_scale;

  // 2. Lateral Acceleration (G-force) Card
  const float a_lat_g = std::abs(state.lateral_accel_mps2) / 9.81f;
  std::array<char, 32> g_buf{};
  (void)snprintf(g_buf.data(), g_buf.size(), "%.2f G", a_lat_g);
  cur_x += draw_telemetry_card_component(
    cur_x, rib_y, rib_h, ui_scale,
    TelemetryCard{"A_LAT", g_buf.data(), (a_lat_g > 0.8f) ? Theme::kAlert : Theme::kTextDim},
    font_bold, font_mono);
  draw_hairline_divider(cur_x, rib_y + 6.0f * ui_scale, rib_h - 12.0f * ui_scale);
  cur_x += Theme::kCardGap * ui_scale;

  // 3. Tracking Error Card
  const auto& tel = robot.get_telemetry();
  std::array<char, 32> err_buf{};
  const char* err_label = "CENTROID";
  if (tel.mode == lf::RobotMode::kRun2Sprint) {
    err_label = "CROSS-TRACK";
    (void)snprintf(err_buf.data(), err_buf.size(), "%+.1f mm", tel.cross_track_error_m * 1000.0f);
  } else {
    (void)snprintf(err_buf.data(), err_buf.size(), "%+.1f mm", tel.centroid_pos_mm);
  }
  Color err_col = Theme::kSuccess;
  if (tel.line_lost) {
    err_col = Theme::kAlert;
  } else if (std::abs(tel.centroid_pos_mm) > 12.0f) {
    err_col = Theme::kWarn;
  }
  cur_x += draw_telemetry_card_component(cur_x, rib_y, rib_h, ui_scale,
                                         TelemetryCard{err_label, err_buf.data(), err_col},
                                         font_bold, font_mono);
  draw_hairline_divider(cur_x, rib_y + 6.0f * ui_scale, rib_h - 12.0f * ui_scale);
  cur_x += Theme::kCardGap * ui_scale;

  // 4. Optical Sensor 8-Eye Mini Bar Card
  {
    const float card_lbl_y = rib_y + 4.0f * ui_scale;
    const float font_lbl_sz = Theme::kFontSmall * ui_scale;
    draw_text_crisp(font_bold, "IR ARRAY", cur_x + Theme::kCardPaddingX * ui_scale, card_lbl_y,
                    font_lbl_sz, Theme::kTextDim);

    const float mini_eye_w = 7.0f * ui_scale;
    const float mini_eye_h = 16.0f * ui_scale;
    const float mini_eye_y = rib_y + 20.0f * ui_scale;
    float eye_x = cur_x + Theme::kCardPaddingX * ui_scale;

    for (size_t i = 0; i < lf::LineSensorConfig::kSensorCount; ++i) {
      const float val = in.ir_raw.at(i);
      const float h = std::clamp(val * mini_eye_h, 2.0f * ui_scale, mini_eye_h);
      DrawRectangle(static_cast<int>(eye_x), static_cast<int>(mini_eye_y),
                    static_cast<int>(mini_eye_w), static_cast<int>(mini_eye_h),
                    Theme::kSurfaceLight);
      const Color eye_col = (val > 0.55f) ? Theme::kSuccess : Theme::kAccent;
      DrawRectangle(static_cast<int>(eye_x), static_cast<int>(mini_eye_y + (mini_eye_h - h)),
                    static_cast<int>(mini_eye_w), static_cast<int>(h), eye_col);
      eye_x += mini_eye_w + 2.0f * ui_scale;
    }
    const float ir_card_w = (eye_x - cur_x) + Theme::kCardPaddingX * ui_scale;
    cur_x += ir_card_w;
  }
  draw_hairline_divider(cur_x, rib_y + 6.0f * ui_scale, rib_h - 12.0f * ui_scale);
  cur_x += Theme::kCardGap * ui_scale;

  // 5. 3-Axis Laser ToF Card
  std::array<char, 48> tof_buf{};
  (void)snprintf(tof_buf.data(), tof_buf.size(), "F:%.2fm  L:%.2fm  R:%.2fm", in.tof_front_m,
                 in.tof_left_m, in.tof_right_m);
  cur_x +=
    draw_telemetry_card_component(cur_x, rib_y, rib_h, ui_scale,
                                  TelemetryCard{
                                    "3-AXIS TOF",
                                    tof_buf.data(),
                                    (in.tof_front_m < 0.15f) ? Theme::kAlert : Theme::kTextDim,
                                  },
                                  font_bold, font_mono);
  draw_hairline_divider(cur_x, rib_y + 6.0f * ui_scale, rib_h - 12.0f * ui_scale);
  cur_x += Theme::kCardGap * ui_scale;

  // 6. Motor Actuator PWM Card
  std::array<char, 32> pwm_buf{};
  (void)snprintf(pwm_buf.data(), pwm_buf.size(), "L:%+.0f%%  R:%+.0f%%", state.left_pwm * 100.0f,
                 state.right_pwm * 100.0f);
  cur_x += draw_telemetry_card_component(
    cur_x, rib_y, rib_h, ui_scale, TelemetryCard{"ACTUATOR PWM", pwm_buf.data(), Theme::kTextMain},
    font_bold, font_mono);
  draw_hairline_divider(cur_x, rib_y + 6.0f * ui_scale, rib_h - 12.0f * ui_scale);
  cur_x += Theme::kCardGap * ui_scale;

  // 7. Total Distance Card
  std::array<char, 32> dist_buf{};
  (void)snprintf(dist_buf.data(), dist_buf.size(), "%.2f m", tel.total_distance_m);
  draw_telemetry_card_component(cur_x, rib_y, rib_h, ui_scale,
                                TelemetryCard{"TOTAL DIST", dist_buf.data(), Theme::kAccent},
                                font_bold, font_mono);
}

void draw_floating_toast(float screen_w, float ui_scale, bool is_setup_mode, const char* toast_msg,
                         const Font& font_bold) {
  const float toast_h = 30.0f * ui_scale;
  const float font_sz = Theme::kFontLabel * ui_scale;
  const Vector2 text_sz = measure_text_dim(font_bold, toast_msg, font_sz);
  const float toast_w = text_sz.x + 36.0f * ui_scale;
  const float toast_x = (screen_w - toast_w) * 0.5f;
  const float toast_y = is_setup_mode ? (96.0f * ui_scale) : (60.0f * ui_scale);
  const Rectangle toast_rect{toast_x, toast_y, toast_w, toast_h};

  DrawRectangleRounded(toast_rect, 0.40f, 4, ColorAlpha(Theme::kSurfaceLight, 0.95f));
  DrawRectangleRoundedLines(toast_rect, 0.40f, 4, Theme::kAccent);
  draw_text_crisp(font_bold, toast_msg, toast_x + 18.0f * ui_scale, toast_y + 7.0f * ui_scale,
                  font_sz, Theme::kTextMain);
}

}  // namespace

void draw_text_crisp(const Font& font, const char* text, float x, float y, float font_size,
                     Color color) {
  if (font.texture.id > 0) {
    DrawTextEx(font, text, Vector2{x, y}, font_size, 1.0f, color);
    return;
  }
  DrawText(text, static_cast<int>(x), static_cast<int>(y), static_cast<int>(font_size), color);
}

HudResult draw_tactical_hud(float screen_w, float screen_h, float ui_scale,
                            const lf::RobotCoordinator& robot, const lf::SensorData& in,
                            SimRobotState& state, size_t obstacle_count,
                            float hovered_obstacle_angle, bool follow_robot, const char* toast_msg,
                            float toast_timer, const Font& font_bold, const Font& font_mono,
                            const char* track_name) {
  (void)obstacle_count;
  (void)hovered_obstacle_angle;
  (void)follow_robot;

  HudResult result{};

  // 1. Top floating status & control pill
  draw_top_control_pill(screen_w, ui_scale, robot, state, track_name, font_bold, font_mono,
                        result.action);

  // 2. Interactive setup toolbar (when in setup mode)
  if (state.is_setup_mode) {
    draw_setup_overlay_toolbar(screen_w, ui_scale, state, font_bold, result.action);
  }

  // 3. Bottom monospace telemetry ribbon
  draw_bottom_telemetry_ribbon(screen_w, screen_h, ui_scale, robot, in, state, font_bold,
                               font_mono);

  // 4. Floating Toast Notification
  if (toast_timer > 0.0f && toast_msg != nullptr && toast_msg[0] != '\0') {
    draw_floating_toast(screen_w, ui_scale, state.is_setup_mode, toast_msg, font_bold);
  }

  return result;
}

void draw_canvas_hover_telemetry(Vector2 mouse_screen, const lf::TrajectoryPoint& pt, size_t pt_idx,
                                 float ui_scale, const Font& font_bold, const Font& font_mono) {
  const float tip_w = 210.0f * ui_scale;
  const float tip_h = 64.0f * ui_scale;
  const float tip_x = mouse_screen.x + 15.0f;
  const float tip_y = mouse_screen.y - tip_h - 10.0f;

  DrawRectangleRounded(Rectangle{tip_x, tip_y, tip_w, tip_h}, 0.20f, 4,
                       ColorAlpha(Theme::kSurface, 0.92f));
  DrawRectangleRoundedLines(Rectangle{tip_x, tip_y, tip_w, tip_h}, 0.20f, 4, Theme::kBorderActive);

  std::array<char, 32> title{};
  (void)snprintf(title.data(), title.size(), "Waypoint #%zu", pt_idx);
  draw_text_crisp(font_bold, title.data(), tip_x + 8.0f * ui_scale, tip_y + 6.0f * ui_scale,
                  12.0f * ui_scale, Theme::kAccent);

  std::array<char, 48> pos_buf{};
  (void)snprintf(pos_buf.data(), pos_buf.size(), "X:%.0f Y:%.0f s:%.2fm", pt.x_mm, pt.y_mm, pt.s_m);
  draw_text_crisp(font_mono, pos_buf.data(), tip_x + 8.0f * ui_scale, tip_y + 24.0f * ui_scale,
                  12.0f * ui_scale, Theme::kTextMain);

  std::array<char, 48> dyn_buf{};
  (void)snprintf(dyn_buf.data(), dyn_buf.size(), "v:%.2fm/s κ:%.2frad/m", pt.target_speed_mps,
                 pt.curvature_rad_m);
  draw_text_crisp(font_mono, dyn_buf.data(), tip_x + 8.0f * ui_scale, tip_y + 42.0f * ui_scale,
                  12.0f * ui_scale, Theme::kWarn);
}

}  // namespace lf::sim
