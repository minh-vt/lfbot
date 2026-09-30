#include "hud.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

namespace lf::sim {

void draw_text_crisp(const Font& font, const char* text, float x, float y, float font_size,
                     Color color) {
  if (font.texture.id > 0) {
    DrawTextEx(font, text, Vector2{x, y}, font_size, 1.0f, color);
    return;
  }
  DrawText(text, static_cast<int>(x), static_cast<int>(y), static_cast<int>(font_size), color);
}

void draw_hud_card(float x, float y, float w, float h, float roundness, Color fill, Color border) {
  DrawRectangleRounded(Rectangle{x, y, w, h}, roundness, 4, fill);
  DrawRectangleRoundedLines(Rectangle{x, y, w, h}, roundness, 4, border);
}

void draw_sidebar_background(float sb_x, float sb_w, int screen_h, float ui_scale) {
  DrawRectangle(static_cast<int>(sb_x), 0, static_cast<int>(sb_w), screen_h,
                Color{18, 23, 33, 255});
  DrawLineEx(Vector2{sb_x, 0}, Vector2{sb_x, static_cast<float>(screen_h)}, 2.0f * ui_scale,
             Color{40, 50, 70, 255});
}

void draw_header_card(float sb_x, float pad_x, float card_w, float ui_scale, float sim_time,
                      bool is_paused, lf::RobotMode mode, bool is_telemetry_mode,
                      uint32_t packets_received, const Font& font_bold, const Font& font_mono) {
  draw_text_crisp(font_bold, "LF-BOT 2026 // RACING SUITE", sb_x + pad_x, 15.0f * ui_scale,
                  21.0f * ui_scale, Color{56, 189, 248, 255});

  const char* mode_title = "MODE: IDLE";
  auto mode_badge_col = Color{100, 116, 139, 255};
  if (is_telemetry_mode) {
    mode_title = "GROUND STATION (LIVE)";
    mode_badge_col = Color{52, 211, 153, 255};
  } else if (mode == lf::RobotMode::kRun1Map) {
    mode_title = "MODE: RUN 1 (MAPPING)";
    mode_badge_col = Color{56, 189, 248, 255};
  } else if (mode == lf::RobotMode::kRun2Sprint) {
    mode_title = "MODE: RUN 2 (C2 SPRINT)";
    mode_badge_col = Color{251, 191, 36, 255};
  } else if (mode == lf::RobotMode::kAutoTuning) {
    mode_title = "MODE: AUTO-TUNING";
    mode_badge_col = Color{192, 132, 252, 255};
  }

  const float pill_w = 230.0f * ui_scale;
  const float pill_h = 26.0f * ui_scale;
  draw_hud_card(sb_x + pad_x, 44.0f * ui_scale, pill_w, pill_h, 0.3f,
                ColorAlpha(mode_badge_col, 0.2f), mode_badge_col);
  draw_text_crisp(font_bold, mode_title, sb_x + pad_x + 12.0f * ui_scale, 49.0f * ui_scale,
                  13.0f * ui_scale, mode_badge_col);

  std::array<char, 32> time_str{};
  if (is_telemetry_mode) {
    (void)snprintf(time_str.data(), time_str.size(), "PKT:%u", packets_received);
  } else {
    (void)snprintf(time_str.data(), time_str.size(), "%05.2fs", sim_time);
  }
  draw_text_crisp(font_mono, time_str.data(), sb_x + card_w - 75.0f * ui_scale, 42.0f * ui_scale,
                  24.0f * ui_scale,
                  is_paused ? Color{248, 113, 113, 255} : Color{240, 246, 252, 255});
}

void draw_telemetry_card(float sb_x, float pad_x, float half_w, float ui_scale, float current_v,
                         float max_speed, const lf::Telemetry& tel, const Font& font_bold) {
  const float card_y = 76.0f * ui_scale;
  const float card_h = 72.0f * ui_scale;

  // Cot Speed
  draw_hud_card(sb_x + pad_x, card_y, half_w, card_h);
  draw_text_crisp(font_bold, "SPEED", sb_x + pad_x + 12.0f * ui_scale, card_y + 7.0f * ui_scale,
                  12.0f * ui_scale, Color{148, 163, 184, 255});
  std::array<char, 32> speed_str{};
  (void)snprintf(speed_str.data(), speed_str.size(), "%.2f", current_v);
  draw_text_crisp(font_bold, speed_str.data(), sb_x + pad_x + 12.0f * ui_scale,
                  card_y + 24.0f * ui_scale, 26.0f * ui_scale, Color{240, 246, 252, 255});
  draw_text_crisp(font_bold, "m/s", sb_x + pad_x + 82.0f * ui_scale, card_y + 34.0f * ui_scale,
                  14.0f * ui_scale, Color{100, 116, 139, 255});

  const float speed_ratio = std::clamp(current_v / max_speed, 0.0f, 1.0f);
  DrawRectangle(static_cast<int>(sb_x + pad_x + 12.0f * ui_scale),
                static_cast<int>(card_y + 57.0f * ui_scale),
                static_cast<int>(half_w - 24.0f * ui_scale), static_cast<int>(5.0f * ui_scale),
                Color{35, 45, 62, 255});
  DrawRectangle(static_cast<int>(sb_x + pad_x + 12.0f * ui_scale),
                static_cast<int>(card_y + 57.0f * ui_scale),
                static_cast<int>((half_w - 24.0f * ui_scale) * speed_ratio),
                static_cast<int>(5.0f * ui_scale), Color{56, 189, 248, 255});

  // Cot Distance
  const float dist_x = sb_x + pad_x + half_w + 12.0f * ui_scale;
  draw_hud_card(dist_x, card_y, half_w, card_h);
  draw_text_crisp(font_bold, "DISTANCE", dist_x + 12.0f * ui_scale, card_y + 7.0f * ui_scale,
                  12.0f * ui_scale, Color{148, 163, 184, 255});
  std::array<char, 32> dist_str{};
  (void)snprintf(dist_str.data(), dist_str.size(), "%.2f", tel.total_distance_m);
  draw_text_crisp(font_bold, dist_str.data(), dist_x + 12.0f * ui_scale, card_y + 24.0f * ui_scale,
                  26.0f * ui_scale, Color{240, 246, 252, 255});
  draw_text_crisp(font_bold, "m", dist_x + 80.0f * ui_scale, card_y + 34.0f * ui_scale,
                  14.0f * ui_scale, Color{100, 116, 139, 255});

  std::array<char, 32> seg_str{};
  (void)snprintf(seg_str.data(), seg_str.size(), "Seg #%d [%s]", tel.current_segment_idx,
                 (tel.fsm_state != 0) ? "BYPASS" : "TRACK");
  draw_text_crisp(font_bold, seg_str.data(), dist_x + 12.0f * ui_scale, card_y + 55.0f * ui_scale,
                  11.0f * ui_scale, Color{251, 191, 36, 255});
}

void draw_centroid_card(float sb_x, float pad_x, float card_w, float ui_scale,
                        const lf::Telemetry& tel, const Font& font_bold) {
  const float card_y = 153.0f * ui_scale;
  const float card_h = 78.0f * ui_scale;
  draw_hud_card(sb_x + pad_x, card_y, card_w, card_h);

  draw_text_crisp(font_bold, "CENTROID TRACKING ERROR", sb_x + pad_x + 12.0f * ui_scale,
                  card_y + 7.0f * ui_scale, 12.0f * ui_scale, Color{148, 163, 184, 255});

  std::array<char, 32> cent_str{};
  (void)snprintf(cent_str.data(), cent_str.size(), "%+.1f mm", tel.centroid_pos_mm);
  auto cent_col = Color{52, 211, 153, 255};
  if (std::abs(tel.centroid_pos_mm) > 15.0f) {
    cent_col = Color{248, 113, 113, 255};
  } else if (std::abs(tel.centroid_pos_mm) > 6.0f) {
    cent_col = Color{251, 191, 36, 255};
  }
  draw_text_crisp(font_bold, cent_str.data(), sb_x + pad_x + 12.0f * ui_scale,
                  card_y + 22.0f * ui_scale, 22.0f * ui_scale, cent_col);

  const char* track_status = tel.line_lost ? "TRUOT VACH (LOST)" : "BAM VACH (LOCKED)";
  const Color track_status_col =
    tel.line_lost ? Color{248, 113, 113, 255} : Color{52, 211, 153, 255};
  draw_text_crisp(font_bold, track_status, sb_x + pad_x + 180.0f * ui_scale,
                  card_y + 26.0f * ui_scale, 13.0f * ui_scale, track_status_col);

  const float slider_x = sb_x + pad_x + 12.0f * ui_scale;
  const float slider_y = card_y + 53.0f * ui_scale;
  const float slider_w = card_w - 24.0f * ui_scale;
  DrawRectangle(static_cast<int>(slider_x), static_cast<int>(slider_y), static_cast<int>(slider_w),
                static_cast<int>(7.0f * ui_scale), Color{35, 45, 62, 255});
  DrawRectangle(static_cast<int>(slider_x + slider_w * 0.5f - 1.5f * ui_scale),
                static_cast<int>(slider_y - 2.0f * ui_scale), static_cast<int>(3.0f * ui_scale),
                static_cast<int>(11.0f * ui_scale), Color{148, 163, 184, 255});

  const float norm_c = std::clamp(tel.centroid_pos_mm / 35.0f, -1.0f, 1.0f);
  const float needle_x = slider_x + (norm_c * 0.5f + 0.5f) * slider_w;
  DrawCircle(static_cast<int>(needle_x), static_cast<int>(slider_y + 3.5f * ui_scale),
             5.0f * ui_scale, cent_col);
}

void draw_sensors_card(float sb_x, float pad_x, float card_w, float ui_scale,
                       const lf::SensorData& in, const Font& font_bold) {
  const float card_y = 236.0f * ui_scale;
  const float card_h = 92.0f * ui_scale;
  draw_hud_card(sb_x + pad_x, card_y, card_w, card_h);

  draw_text_crisp(font_bold, "ANALOG IR SENSOR ARRAY (8 EYES)", sb_x + pad_x + 12.0f * ui_scale,
                  card_y + 7.0f * ui_scale, 12.0f * ui_scale, Color{148, 163, 184, 255});

  const float bar_margin = 12.0f * ui_scale;
  const float total_bars_w = card_w - 2.0f * bar_margin;
  const float bar_w = total_bars_w / 8.0f;
  const float bar_inner_w = bar_w - 8.0f * ui_scale;
  const float max_bar_h = 38.0f * ui_scale;

  for (size_t i = 0; i < lf::LineSensorConfig::kSensorCount; ++i) {
    const float bx = sb_x + pad_x + bar_margin + static_cast<float>(i) * bar_w + 4.0f * ui_scale;
    const float b_val = in.ir_raw.at(i);
    const float b_h = std::clamp(b_val * max_bar_h, 2.0f * ui_scale, max_bar_h);

    DrawRectangle(static_cast<int>(bx), static_cast<int>(card_y + 28.0f * ui_scale),
                  static_cast<int>(bar_inner_w), static_cast<int>(max_bar_h),
                  Color{35, 45, 62, 255});

    Color col_bar = ColorAlpha(Color{56, 189, 248, 255}, 0.35f + 0.65f * b_val);
    if (b_val > 0.60f) {
      col_bar = Color{52, 211, 153, 255};
    }
    DrawRectangle(static_cast<int>(bx),
                  static_cast<int>(card_y + 28.0f * ui_scale + (max_bar_h - b_h)),
                  static_cast<int>(bar_inner_w), static_cast<int>(b_h), col_bar);

    std::array<char, 8> eye_lbl{};
    (void)snprintf(eye_lbl.data(), eye_lbl.size(), "S%zu", i);
    draw_text_crisp(font_bold, eye_lbl.data(), bx + 2.0f * ui_scale, card_y + 72.0f * ui_scale,
                    10.0f * ui_scale, Color{100, 116, 139, 255});
  }
}

void draw_motor_pwm_card(float sb_x, float pad_x, float card_w, float ui_scale, float left_pwm,
                         float right_pwm, const lf::Telemetry& tel, const Font& font_bold,
                         const Font& font_mono) {
  const float card_y = 333.0f * ui_scale;
  const float card_h = 78.0f * ui_scale;
  draw_hud_card(sb_x + pad_x, card_y, card_w, card_h);

  draw_text_crisp(font_bold, "DRIVETRAIN MOTORS & ENCODERS", sb_x + pad_x + 12.0f * ui_scale,
                  card_y + 7.0f * ui_scale, 12.0f * ui_scale, Color{148, 163, 184, 255});

  const float pwm_bar_w = card_w - 230.0f * ui_scale;
  const float pwm_bar_x = sb_x + pad_x + 210.0f * ui_scale;

  // Left Motor
  std::array<char, 48> lp_str{};
  (void)snprintf(lp_str.data(), lp_str.size(), "L: %+3.0f%% | %.2fm/s | %dt", left_pwm * 100.0f,
                 tel.wheel_l.actual_speed_mps, tel.wheel_l.enc_ticks);
  draw_text_crisp(font_mono, lp_str.data(), sb_x + pad_x + 12.0f * ui_scale,
                  card_y + 24.0f * ui_scale, 11.5f * ui_scale, Color{240, 246, 252, 255});
  DrawRectangle(static_cast<int>(pwm_bar_x), static_cast<int>(card_y + 26.0f * ui_scale),
                static_cast<int>(pwm_bar_w), static_cast<int>(9.0f * ui_scale),
                Color{35, 45, 62, 255});
  const float l_bar_w = std::clamp(std::abs(left_pwm) * (pwm_bar_w * 0.5f), 0.0f, pwm_bar_w * 0.5f);
  const float center_bar_x = pwm_bar_x + pwm_bar_w * 0.5f;
  if (left_pwm >= 0.0f) {
    DrawRectangle(static_cast<int>(center_bar_x), static_cast<int>(card_y + 26.0f * ui_scale),
                  static_cast<int>(l_bar_w), static_cast<int>(9.0f * ui_scale),
                  Color{56, 189, 248, 255});
  } else {
    DrawRectangle(static_cast<int>(center_bar_x - l_bar_w),
                  static_cast<int>(card_y + 26.0f * ui_scale), static_cast<int>(l_bar_w),
                  static_cast<int>(9.0f * ui_scale), Color{248, 113, 113, 255});
  }

  // Right Motor
  std::array<char, 48> rp_str{};
  (void)snprintf(rp_str.data(), rp_str.size(), "R: %+3.0f%% | %.2fm/s | %dt", right_pwm * 100.0f,
                 tel.wheel_r.actual_speed_mps, tel.wheel_r.enc_ticks);
  draw_text_crisp(font_mono, rp_str.data(), sb_x + pad_x + 12.0f * ui_scale,
                  card_y + 46.0f * ui_scale, 11.5f * ui_scale, Color{240, 246, 252, 255});
  DrawRectangle(static_cast<int>(pwm_bar_x), static_cast<int>(card_y + 48.0f * ui_scale),
                static_cast<int>(pwm_bar_w), static_cast<int>(9.0f * ui_scale),
                Color{35, 45, 62, 255});
  const float r_bar_w =
    std::clamp(std::abs(right_pwm) * (pwm_bar_w * 0.5f), 0.0f, pwm_bar_w * 0.5f);
  if (right_pwm >= 0.0f) {
    DrawRectangle(static_cast<int>(center_bar_x), static_cast<int>(card_y + 48.0f * ui_scale),
                  static_cast<int>(r_bar_w), static_cast<int>(9.0f * ui_scale),
                  Color{251, 191, 36, 255});
  } else {
    DrawRectangle(static_cast<int>(center_bar_x - r_bar_w),
                  static_cast<int>(card_y + 48.0f * ui_scale), static_cast<int>(r_bar_w),
                  static_cast<int>(9.0f * ui_scale), Color{248, 113, 113, 255});
  }
}

void draw_tof_card(float sb_x, float pad_x, float card_w, float ui_scale, const lf::SensorData& in,
                   size_t obstacle_count, float hovered_angle_deg, const Font& font_bold,
                   const Font& font_mono) {
  const float card_y = 416.0f * ui_scale;
  const float card_h = 74.0f * ui_scale;
  draw_hud_card(sb_x + pad_x, card_y, card_w, card_h);

  draw_text_crisp(font_bold, "OBSTACLE & ToF SENSORS (3-AXIS)", sb_x + pad_x + 12.0f * ui_scale,
                  card_y + 7.0f * ui_scale, 12.0f * ui_scale, Color{245, 158, 11, 255});

  std::array<char, 32> obs_cnt_str{};
  if (hovered_angle_deg >= 0.0f) {
    (void)snprintf(obs_cnt_str.data(), obs_cnt_str.size(), "Rot: %.0f°", hovered_angle_deg);
  } else {
    (void)snprintf(obs_cnt_str.data(), obs_cnt_str.size(), "%zu Objects", obstacle_count);
  }
  draw_text_crisp(font_mono, obs_cnt_str.data(), sb_x + card_w - 70.0f * ui_scale,
                  card_y + 7.0f * ui_scale, 11.5f * ui_scale, Color{148, 163, 184, 255});

  const float tof_col_w = (card_w - 24.0f * ui_scale) / 3.0f;

  const float tx1 = sb_x + pad_x + 12.0f * ui_scale;
  draw_text_crisp(font_bold, "LEFT (-90°)", tx1, card_y + 24.0f * ui_scale, 10.5f * ui_scale,
                  Color{148, 163, 184, 255});
  std::array<char, 16> l_str{};
  (void)snprintf(l_str.data(), l_str.size(), "%.2f m", in.tof_left_m);
  draw_text_crisp(font_mono, l_str.data(), tx1, card_y + 36.0f * ui_scale, 15.0f * ui_scale,
                  Color{251, 191, 36, 255});

  const float tx2 = tx1 + tof_col_w;
  draw_text_crisp(font_bold, "FRONT (0°)", tx2, card_y + 24.0f * ui_scale, 10.5f * ui_scale,
                  Color{148, 163, 184, 255});
  std::array<char, 16> f_str{};
  (void)snprintf(f_str.data(), f_str.size(), "%.2f m", in.tof_front_m);
  const Color f_txt_col =
    (in.tof_front_m < 0.15f) ? Color{248, 113, 113, 255} : Color{56, 189, 248, 255};
  draw_text_crisp(font_mono, f_str.data(), tx2, card_y + 36.0f * ui_scale, 15.0f * ui_scale,
                  f_txt_col);

  const float tx3 = tx2 + tof_col_w;
  draw_text_crisp(font_bold, "RIGHT (+90°)", tx3, card_y + 24.0f * ui_scale, 10.5f * ui_scale,
                  Color{148, 163, 184, 255});
  std::array<char, 16> r_str{};
  (void)snprintf(r_str.data(), r_str.size(), "%.2f m", in.tof_right_m);
  draw_text_crisp(font_mono, r_str.data(), tx3, card_y + 36.0f * ui_scale, 15.0f * ui_scale,
                  Color{192, 132, 252, 255});

  draw_text_crisp(font_bold, "Cuon chuot / [Q],[E]: Xoay  |  [Click/Drag]: Di chuyen  |  [O]: Them",
                  sb_x + pad_x + 12.0f * ui_scale, card_y + 56.0f * ui_scale, 10.5f * ui_scale,
                  Color{100, 116, 139, 255});
}

void draw_lap_comparison_card(float sb_x, float pad_x, float card_w, float ui_scale,
                              const SimRobotState& state, const Font& font_bold,
                              const Font& font_mono) {
  const float card_y = 495.0f * ui_scale;
  const float card_h = 76.0f * ui_scale;
  draw_hud_card(sb_x + pad_x, card_y, card_w, card_h);

  draw_text_crisp(font_bold, "LAP TIME & TRAJECTORY COMPARISON", sb_x + pad_x + 12.0f * ui_scale,
                  card_y + 7.0f * ui_scale, 12.0f * ui_scale, Color{52, 211, 153, 255});

  std::array<char, 64> r1_str{};
  if (state.has_run1_data || state.run1_time_s > 0.01f) {
    (void)snprintf(r1_str.data(), r1_str.size(), "Run 1 (Map/Scout) : %05.2fs", state.run1_time_s);
  } else {
    (void)snprintf(r1_str.data(), r1_str.size(), "Run 1 (Map/Scout) : --.--s (Press [1] to Map)");
  }
  draw_text_crisp(font_mono, r1_str.data(), sb_x + pad_x + 12.0f * ui_scale,
                  card_y + 24.0f * ui_scale, 12.0f * ui_scale, Color{56, 189, 248, 255});

  std::array<char, 80> r2_str{};
  if (state.has_run2_data || state.run2_time_s > 0.01f) {
    const float delta_s = state.run2_time_s - state.run1_time_s;
    const float pct = (state.run1_time_s > 0.01f) ? ((delta_s / state.run1_time_s) * 100.0f) : 0.0f;
    (void)snprintf(r2_str.data(), r2_str.size(), "Run 2 (C2 Sprint) : %05.2fs [%+.2fs (%+.1f%%)]",
                   state.run2_time_s, delta_s, pct);
  } else {
    (void)snprintf(r2_str.data(), r2_str.size(),
                   "Run 2 (C2 Sprint) : --.--s (Press [2] to Sprint)");
  }
  draw_text_crisp(font_mono, r2_str.data(), sb_x + pad_x + 12.0f * ui_scale,
                  card_y + 40.0f * ui_scale, 12.0f * ui_scale, Color{251, 191, 36, 255});

  draw_text_crisp(font_bold, "-- Vet Run 1 (Cyan)  |  == Quy dao dua C2 (Ho phach -> Ngoc luc bao)",
                  sb_x + pad_x + 12.0f * ui_scale, card_y + 58.0f * ui_scale, 9.5f * ui_scale,
                  Color{148, 163, 184, 255});
}

void draw_tactical_card(float sb_x, float pad_x, float card_w, float ui_scale,
                        const lf::RobotCoordinator& robot, const Font& font_bold,
                        const Font& font_mono) {
  const float card_y = 576.0f * ui_scale;
  const float card_h = 224.0f * ui_scale;
  draw_hud_card(sb_x + pad_x, card_y, card_w, card_h, 0.15f);

  draw_text_crisp(font_bold, "TACTICAL PARAMETERS (LIVE TUNING)", sb_x + pad_x + 12.0f * ui_scale,
                  card_y + 7.0f * ui_scale, 12.0f * ui_scale, Color{192, 132, 252, 255});

  const bool is_opt = robot.get_profiler().is_optimized();
  std::array<char, 32> map_badge{};
  if (is_opt) {
    (void)snprintf(map_badge.data(), map_badge.size(), "MAP: OPT (%d segs)",
                   robot.get_profiler().get_segment_count());
  } else {
    (void)snprintf(map_badge.data(), map_badge.size(), "MAP: UNMAPPED");
  }
  draw_text_crisp(font_bold, map_badge.data(), sb_x + card_w - 95.0f * ui_scale,
                  card_y + 7.0f * ui_scale, 11.0f * ui_scale,
                  is_opt ? Color{52, 211, 153, 255} : Color{251, 191, 36, 255});

  float cur_kp = 0.0f;
  float cur_ki = 0.0f;
  float cur_kd = 0.0f;
  robot.get_line_pid_gains(cur_kp, cur_ki, cur_kd);
  const float cur_v_map = robot.get_config().profiler.safe_map_speed_mps;
  const float cur_v_sprint = robot.get_config().profiler.max_straight_speed_mps;
  float cur_obs_front = 0.0f;
  float cur_obs_clear = 0.0f;
  float cur_obs_len = 0.0f;
  robot.get_obstacle_parameters(cur_obs_front, cur_obs_clear, cur_obs_len);

  float param_y = card_y + 24.0f * ui_scale;
  const float param_gap = 18.0f * ui_scale;
  const float hotkey_x = sb_x + card_w - 75.0f * ui_scale;
  std::array<char, 64> p_line{};

  (void)snprintf(p_line.data(), p_line.size(), "Line Kp : %.4f", cur_kp);
  draw_text_crisp(font_mono, p_line.data(), sb_x + pad_x + 12.0f * ui_scale, param_y,
                  12.5f * ui_scale, Color{240, 246, 252, 255});
  draw_text_crisp(font_bold, "[U] / [J]", hotkey_x, param_y, 11.5f * ui_scale,
                  Color{56, 189, 248, 255});

  param_y += param_gap;
  (void)snprintf(p_line.data(), p_line.size(), "Line Kd : %.4f", cur_kd);
  draw_text_crisp(font_mono, p_line.data(), sb_x + pad_x + 12.0f * ui_scale, param_y,
                  12.5f * ui_scale, Color{240, 246, 252, 255});
  draw_text_crisp(font_bold, "[I] / [K]", hotkey_x, param_y, 11.5f * ui_scale,
                  Color{56, 189, 248, 255});

  param_y += param_gap;
  (void)snprintf(p_line.data(), p_line.size(), "Line Ki : %.5f", cur_ki);
  draw_text_crisp(font_mono, p_line.data(), sb_x + pad_x + 12.0f * ui_scale, param_y,
                  12.5f * ui_scale, Color{240, 246, 252, 255});
  draw_text_crisp(font_bold, "[P] / [L]", hotkey_x, param_y, 11.5f * ui_scale,
                  Color{56, 189, 248, 255});

  param_y += param_gap + 2.0f * ui_scale;
  DrawLine(static_cast<int>(sb_x + pad_x + 12.0f * ui_scale),
           static_cast<int>(param_y - 2.0f * ui_scale),
           static_cast<int>(sb_x + card_w + pad_x - 12.0f * ui_scale),
           static_cast<int>(param_y - 2.0f * ui_scale), Color{40, 50, 70, 255});

  (void)snprintf(p_line.data(), p_line.size(), "Map Speed V_map   : %.2f m/s", cur_v_map);
  draw_text_crisp(font_mono, p_line.data(), sb_x + pad_x + 12.0f * ui_scale, param_y,
                  12.5f * ui_scale, Color{240, 246, 252, 255});
  draw_text_crisp(font_bold, "[E] / [D]", hotkey_x, param_y, 11.5f * ui_scale,
                  Color{52, 211, 153, 255});

  param_y += param_gap;
  (void)snprintf(p_line.data(), p_line.size(), "Sprint Speed V_top: %.2f m/s", cur_v_sprint);
  draw_text_crisp(font_mono, p_line.data(), sb_x + pad_x + 12.0f * ui_scale, param_y,
                  12.5f * ui_scale, Color{240, 246, 252, 255});
  draw_text_crisp(font_bold, "[W] / [S]", hotkey_x, param_y, 11.5f * ui_scale,
                  Color{251, 191, 36, 255});

  param_y += param_gap;
  (void)snprintf(p_line.data(), p_line.size(), "Obs Detect Dist   : %.2f m", cur_obs_front);
  draw_text_crisp(font_mono, p_line.data(), sb_x + pad_x + 12.0f * ui_scale, param_y,
                  12.5f * ui_scale, Color{240, 246, 252, 255});
  draw_text_crisp(font_bold, "[Y] / [H]", hotkey_x, param_y, 11.5f * ui_scale,
                  Color{248, 113, 113, 255});

  param_y += param_gap;
  (void)snprintf(p_line.data(), p_line.size(), "Obs Clearance     : %.2f m", cur_obs_clear);
  draw_text_crisp(font_mono, p_line.data(), sb_x + pad_x + 12.0f * ui_scale, param_y,
                  12.5f * ui_scale, Color{240, 246, 252, 255});
  draw_text_crisp(font_bold, "[B] / [G]", hotkey_x, param_y, 11.5f * ui_scale,
                  Color{251, 191, 36, 255});

  param_y += param_gap;
  (void)snprintf(p_line.data(), p_line.size(), "Obs Arc Length    : %.2f m", cur_obs_len);
  draw_text_crisp(font_mono, p_line.data(), sb_x + pad_x + 12.0f * ui_scale, param_y,
                  12.5f * ui_scale, Color{240, 246, 252, 255});
  draw_text_crisp(font_bold, "[N] / [V]", hotkey_x, param_y, 11.5f * ui_scale,
                  Color{52, 211, 153, 255});
}

void draw_toast_notification(float sb_x, float pad_x, float card_w, float ui_scale,
                             const char* toast_msg, const Font& font_bold) {
  const float card_y = 822.0f * ui_scale;
  const float card_h = 30.0f * ui_scale;
  draw_hud_card(sb_x + pad_x, card_y, card_w, card_h, 0.25f,
                ColorAlpha(Color{56, 189, 248, 255}, 0.25f), Color{56, 189, 248, 255});
  draw_text_crisp(font_bold, toast_msg, sb_x + pad_x + 12.0f * ui_scale, card_y + 8.0f * ui_scale,
                  12.0f * ui_scale, Color{240, 246, 252, 255});
}

void draw_controls_card(float sb_x, float pad_x, float card_w, float view_h, float ui_scale,
                        bool follow_robot, const Font& font_bold) {
  const float card_y = 856.0f * ui_scale;
  const float card_h = view_h - card_y - 10.0f * ui_scale;
  draw_hud_card(sb_x + pad_x, card_y, card_w, card_h, 0.15f, Color{22, 28, 40, 255},
                Color{45, 56, 78, 255});

  draw_text_crisp(font_bold, "SYSTEM CONTROLS", sb_x + pad_x + 12.0f * ui_scale,
                  card_y + 7.0f * ui_scale, 12.0f * ui_scale, Color{148, 163, 184, 255});

  float ky = card_y + 24.0f * ui_scale;
  const float col_step = card_w / 3.0f;
  draw_text_crisp(font_bold, "[1] Run 1 (Map)", sb_x + pad_x + 12.0f * ui_scale, ky,
                  11.5f * ui_scale, Color{56, 189, 248, 255});
  draw_text_crisp(font_bold, "[2] Run 2 (Sprint)", sb_x + pad_x + 12.0f * ui_scale + col_step, ky,
                  11.5f * ui_scale, Color{251, 191, 36, 255});
  draw_text_crisp(font_bold, "[T] Auto-Tune", sb_x + pad_x + 12.0f * ui_scale + 2.0f * col_step, ky,
                  11.5f * ui_scale, Color{192, 132, 252, 255});

  ky += 18.0f * ui_scale;
  draw_text_crisp(font_bold, "[SPACE] Play/Pause", sb_x + pad_x + 12.0f * ui_scale, ky,
                  11.5f * ui_scale, Color{52, 211, 153, 255});
  draw_text_crisp(font_bold, "[R] Reset Pose", sb_x + pad_x + 12.0f * ui_scale + col_step, ky,
                  11.5f * ui_scale, Color{248, 113, 113, 255});
  draw_text_crisp(font_bold, "[Shift+R] All", sb_x + pad_x + 12.0f * ui_scale + 2.0f * col_step, ky,
                  11.5f * ui_scale, Color{248, 113, 113, 200});

  ky += 18.0f * ui_scale;
  draw_text_crisp(font_bold, "[C] Follow Cam", sb_x + pad_x + 12.0f * ui_scale, ky,
                  11.5f * ui_scale,
                  follow_robot ? Color{52, 211, 153, 255} : Color{148, 163, 184, 255});
  draw_text_crisp(font_bold, "[F] Fit Arena", sb_x + pad_x + 12.0f * ui_scale + col_step, ky,
                  11.5f * ui_scale, Color{148, 163, 184, 255});
  draw_text_crisp(font_bold, "[R-Drag] Pan", sb_x + pad_x + 12.0f * ui_scale + 2.0f * col_step, ky,
                  11.5f * ui_scale, Color{148, 163, 184, 255});

  ky += 18.0f * ui_scale;
  draw_text_crisp(font_bold, "[Wheel/Q,E] Xoay vat", sb_x + pad_x + 12.0f * ui_scale, ky,
                  11.5f * ui_scale, Color{245, 158, 11, 255});
  draw_text_crisp(font_bold, "[Click/Drag] Di chuyen", sb_x + pad_x + 12.0f * ui_scale + col_step,
                  ky, 11.5f * ui_scale, Color{245, 158, 11, 255});
  draw_text_crisp(font_bold, "[X] Xoa tat ca", sb_x + pad_x + 12.0f * ui_scale + 2.0f * col_step,
                  ky, 11.5f * ui_scale, Color{248, 113, 113, 255});

  ky += 18.0f * ui_scale;
  draw_text_crisp(font_bold, "[F4] Ground Stn", sb_x + pad_x + 12.0f * ui_scale, ky,
                  11.5f * ui_scale, Color{52, 211, 153, 255});
  draw_text_crisp(font_bold, "[F3] Auto-Scale", sb_x + pad_x + 12.0f * ui_scale + col_step, ky,
                  11.5f * ui_scale, Color{56, 189, 248, 255});
  draw_text_crisp(font_bold, "[Enter] Chot C2", sb_x + pad_x + 12.0f * ui_scale + 2.0f * col_step,
                  ky, 11.5f * ui_scale, Color{251, 191, 36, 255});
}

}  // namespace lf::sim
