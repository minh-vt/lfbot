#include "hud.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

#include "config/constants.hpp"

namespace lf::sim {

namespace {

constexpr float kWheelDiameterM = 0.034f;
constexpr float kCircumferenceM = constants::kPi * kWheelDiameterM;  // 0.106814 m
constexpr float kEncoderTicksPerRev = 560.0f;

void draw_panel(Rectangle rect) {
  DrawRectangleRounded(rect, 0.08f, 4, HudTheme::kSurface);
  DrawRectangleRoundedLines(rect, 0.08f, 4, HudTheme::kBorder);
}

void draw_hairline(float x, float y, float w) {
  DrawLine(static_cast<int>(x), static_cast<int>(y), static_cast<int>(x + w), static_cast<int>(y),
           HudTheme::kBorder);
}

void draw_bipolar_indicator(float bar_x, float bar_y, float bar_w, float norm_val, Color dev_col,
                            float scale) {
  DrawRectangle(static_cast<int>(bar_x), static_cast<int>(bar_y), static_cast<int>(bar_w),
                static_cast<int>(6.0f * scale), HudTheme::kSurfaceLight);
  DrawRectangle(static_cast<int>(bar_x + bar_w * 0.5f - 1.0f * scale),
                static_cast<int>(bar_y - 2.0f * scale), static_cast<int>(2.0f * scale),
                static_cast<int>(10.0f * scale), HudTheme::kBorderLight);

  const float norm_c = std::clamp(norm_val, -1.0f, 1.0f);
  const float needle_x = bar_x + (norm_c * 0.5f + 0.5f) * bar_w;
  DrawCircle(static_cast<int>(needle_x), static_cast<int>(bar_y + 3.0f * scale), 4.5f * scale,
             dev_col);
}

struct LayoutCursor {
  float x = 0.0f;
  float y = 0.0f;
  float width = 0.0f;
  float scale = 1.0f;

  void advance(float height_px, float margin_px = 8.0f) { y += (height_px + margin_px) * scale; }
};

void draw_header_section(LayoutCursor& layout, const SimRobotState& state, lf::RobotMode mode,
                         const Font& font_bold, const Font& font_mono) {
  const float h = 38.0f;
  draw_panel(Rectangle{layout.x, layout.y, layout.width, h * layout.scale});

  draw_text_crisp(font_bold, "LF-BOT TELEMETRY", layout.x + 12.0f * layout.scale,
                  layout.y + 11.0f * layout.scale, 15.0f * layout.scale, HudTheme::kAccent);

  const char* badge_text = "IDLE";
  Color badge_col = HudTheme::kTextDim;
  if (state.is_crashed) {
    badge_text = "FAILED: CRASH";
    badge_col = HudTheme::kAlert;
  } else if (state.is_telemetry_mode) {
    badge_text = "LIVE GROUND";
    badge_col = HudTheme::kSuccess;
  } else if (state.is_paused) {
    badge_text = "PAUSED";
    badge_col = HudTheme::kAlert;
  } else if (mode == lf::RobotMode::kAutoTune) {
    badge_text = "AUTO-TUNE";
    badge_col = HudTheme::kSuccess;
  } else if (mode == lf::RobotMode::kRun1Map) {
    badge_text = "RUN 1: MAP";
    badge_col = HudTheme::kAccent;
  } else if (mode == lf::RobotMode::kRun2Sprint) {
    badge_text = "RUN 2: C2 SPRINT";
    badge_col = HudTheme::kWarn;
  }

  const float badge_x = layout.x + 165.0f * layout.scale;
  const float badge_w = 110.0f * layout.scale;
  const float badge_h = 22.0f * layout.scale;
  DrawRectangleRounded(Rectangle{badge_x, layout.y + 8.0f * layout.scale, badge_w, badge_h}, 0.3f,
                       4, ColorAlpha(badge_col, 0.18f));
  DrawRectangleRoundedLines(Rectangle{badge_x, layout.y + 8.0f * layout.scale, badge_w, badge_h},
                            0.3f, 4, badge_col);
  draw_text_crisp(font_bold, badge_text, badge_x + 6.0f * layout.scale,
                  layout.y + 12.0f * layout.scale, 11.5f * layout.scale, badge_col);

  std::array<char, 32> time_str{};
  if (state.is_telemetry_mode) {
    (void)snprintf(time_str.data(), time_str.size(), "PKT %u", state.packets_received);
  } else {
    (void)snprintf(time_str.data(), time_str.size(), "%05.2fs", state.sim_time);
  }
  draw_text_crisp(font_mono, time_str.data(), layout.x + layout.width - 110.0f * layout.scale,
                  layout.y + 7.0f * layout.scale, 24.0f * layout.scale,
                  state.is_paused ? HudTheme::kAlert : HudTheme::kTextMain);

  layout.advance(h);

  if (state.is_crashed) {
    const float alert_h = 28.0f;
    const Rectangle alert_rect{layout.x, layout.y, layout.width, alert_h * layout.scale};
    DrawRectangleRounded(alert_rect, 0.2f, 4, ColorAlpha(HudTheme::kAlert, 0.22f));
    DrawRectangleRoundedLines(alert_rect, 0.2f, 4, HudTheme::kAlert);
    draw_text_crisp(font_bold, "CRASH DETECTED: ROBOT HALTED (FAIL)",
                    layout.x + 12.0f * layout.scale, layout.y + 7.0f * layout.scale,
                    12.5f * layout.scale, HudTheme::kAlert);
    layout.advance(alert_h);
  }
}

void draw_hero_telemetry_section(LayoutCursor& layout, float current_v, float max_v, float a_long,
                                 float a_lat, float corner_grip_mu, const lf::Telemetry& tel,
                                 const Font& font_bold, const Font& font_mono) {
  const float h = 84.0f;
  const Rectangle rect{layout.x, layout.y, layout.width, h * layout.scale};
  draw_panel(rect);

  const float radar_w = 88.0f * layout.scale;
  const float col_w = (layout.width - radar_w - 2.0f * layout.scale) * 0.5f;

  // Col 1: Speed
  const float c1_x = layout.x + 12.0f * layout.scale;
  draw_text_crisp(font_bold, "SPEED", c1_x, layout.y + 8.0f * layout.scale, 12.0f * layout.scale,
                  HudTheme::kTextMuted);
  std::array<char, 16> spd_val{};
  (void)snprintf(spd_val.data(), spd_val.size(), "%.2f", current_v);
  draw_text_crisp(font_mono, spd_val.data(), c1_x, layout.y + 24.0f * layout.scale,
                  26.0f * layout.scale, HudTheme::kTextMain);
  draw_text_crisp(font_bold, "m/s", c1_x + 65.0f * layout.scale, layout.y + 35.0f * layout.scale,
                  13.0f * layout.scale, HudTheme::kTextDim);

  const int wheel_rpm = static_cast<int>(std::round((current_v * 60.0f) / kCircumferenceM));
  const float enc_khz = (current_v * kEncoderTicksPerRev / kCircumferenceM) * 0.001f;
  std::array<char, 32> spd_sub{};
  (void)snprintf(spd_sub.data(), spd_sub.size(), "%d RPM · %.1f kHz", wheel_rpm, enc_khz);
  draw_text_crisp(font_mono, spd_sub.data(), c1_x, layout.y + 58.0f * layout.scale,
                  12.0f * layout.scale, HudTheme::kAccent);

  const float bar_w = col_w - 24.0f * layout.scale;
  const float spd_ratio = std::clamp(current_v / std::max(0.1f, max_v), 0.0f, 1.0f);
  DrawRectangle(static_cast<int>(c1_x), static_cast<int>(layout.y + 73.0f * layout.scale),
                static_cast<int>(bar_w), static_cast<int>(4.0f * layout.scale),
                HudTheme::kSurfaceLight);
  DrawRectangle(static_cast<int>(c1_x), static_cast<int>(layout.y + 73.0f * layout.scale),
                static_cast<int>(bar_w * spd_ratio), static_cast<int>(4.0f * layout.scale),
                HudTheme::kAccent);

  // Hairline 1
  DrawLine(static_cast<int>(layout.x + col_w), static_cast<int>(layout.y + 8.0f * layout.scale),
           static_cast<int>(layout.x + col_w),
           static_cast<int>(layout.y + (h - 8.0f) * layout.scale), HudTheme::kBorder);

  // Col 2: Distance
  const float c2_x = layout.x + col_w + 12.0f * layout.scale;
  draw_text_crisp(font_bold, "DISTANCE", c2_x, layout.y + 8.0f * layout.scale, 12.0f * layout.scale,
                  HudTheme::kTextMuted);
  std::array<char, 16> dst_val{};
  (void)snprintf(dst_val.data(), dst_val.size(), "%.2f", tel.total_distance_m);
  draw_text_crisp(font_mono, dst_val.data(), c2_x, layout.y + 24.0f * layout.scale,
                  26.0f * layout.scale, HudTheme::kTextMain);
  draw_text_crisp(font_bold, "m", c2_x + 65.0f * layout.scale, layout.y + 35.0f * layout.scale,
                  13.0f * layout.scale, HudTheme::kTextDim);

  std::array<char, 32> seg_sub{};
  (void)snprintf(seg_sub.data(), seg_sub.size(), "Seg #%d [%s]", tel.current_segment_idx,
                 (tel.bypass_state != 0) ? "BYPASS" : "TRACK");
  draw_text_crisp(font_mono, seg_sub.data(), c2_x, layout.y + 58.0f * layout.scale,
                  12.0f * layout.scale,
                  (tel.bypass_state != 0) ? HudTheme::kWarn : HudTheme::kTextMuted);

  // Hairline 2
  const float h2_x = layout.x + layout.width - radar_w;
  DrawLine(static_cast<int>(h2_x), static_cast<int>(layout.y + 8.0f * layout.scale),
           static_cast<int>(h2_x), static_cast<int>(layout.y + (h - 8.0f) * layout.scale),
           HudTheme::kBorder);

  // Col 3: G-G Radar
  const float c3_x = h2_x;
  draw_text_crisp(font_bold, "G-G RADAR", c3_x + 8.0f * layout.scale,
                  layout.y + 8.0f * layout.scale, 11.5f * layout.scale, HudTheme::kTextMuted);

  const float cx = c3_x + radar_w * 0.5f;
  const float cy = layout.y + 44.0f * layout.scale;
  const float r_limit = 22.0f * layout.scale;

  DrawCircleLines(static_cast<int>(cx), static_cast<int>(cy), r_limit, HudTheme::kBorderLight);
  DrawLine(static_cast<int>(cx - r_limit), static_cast<int>(cy), static_cast<int>(cx + r_limit),
           static_cast<int>(cy), HudTheme::kBorder);
  DrawLine(static_cast<int>(cx), static_cast<int>(cy - r_limit), static_cast<int>(cx),
           static_cast<int>(cy + r_limit), HudTheme::kBorder);

  const float g = 9.81f;
  const float max_g = std::max(0.2f, corner_grip_mu * g);
  const float a_total = std::sqrt(a_lat * a_lat + a_long * a_long);
  const float px = cx + std::clamp(a_lat / max_g, -1.25f, 1.25f) * r_limit;
  const float py = cy - std::clamp(a_long / max_g, -1.25f, 1.25f) * r_limit;

  const Color dot_col = (a_total > max_g) ? HudTheme::kAlert : HudTheme::kAccent;
  DrawLineEx(Vector2{cx, cy}, Vector2{px, py}, 1.5f * layout.scale, ColorAlpha(dot_col, 0.6f));
  DrawCircle(static_cast<int>(px), static_cast<int>(py), 3.0f * layout.scale, dot_col);

  std::array<char, 16> g_str{};
  (void)snprintf(g_str.data(), g_str.size(), "%.2fG", a_total / g);
  draw_text_crisp(font_mono, g_str.data(), c3_x + 8.0f * layout.scale,
                  layout.y + 68.0f * layout.scale, 12.0f * layout.scale, dot_col);

  layout.advance(h);
}

void draw_tracking_and_sensors_section(LayoutCursor& layout, const lf::Telemetry& tel,
                                       const lf::SensorData& in, const Font& font_bold,
                                       const Font& font_mono) {
  const float h = 108.0f;
  draw_panel(Rectangle{layout.x, layout.y, layout.width, h * layout.scale});

  if (tel.mode == lf::RobotMode::kRun2Sprint) {
    // RUN 2 SPRINT: TRAJECTORY-CENTRIC COCKPIT (STANLEY KINEMATIC TRACKER)
    draw_text_crisp(font_bold, "CROSS-TRACK ERROR e(t)", layout.x + 12.0f * layout.scale,
                    layout.y + 8.0f * layout.scale, 12.0f * layout.scale, HudTheme::kTextMuted);

    const float cte_mm = tel.cross_track_error_m * 1000.0f;
    std::array<char, 32> dev_str{};
    (void)snprintf(dev_str.data(), dev_str.size(), "%+.1f mm", cte_mm);
    Color dev_col = HudTheme::kSuccess;
    if (std::abs(cte_mm) > 25.0f) {
      dev_col = HudTheme::kAlert;
    } else if (std::abs(cte_mm) > 10.0f) {
      dev_col = HudTheme::kWarn;
    }
    draw_text_crisp(font_mono, dev_str.data(), layout.x + 12.0f * layout.scale,
                    layout.y + 22.0f * layout.scale, 18.0f * layout.scale, dev_col);

    const char* track_status = (std::abs(cte_mm) <= 15.0f) ? "ON TRAJECTORY" : "CORRIDOR DRIFT";
    draw_text_crisp(font_bold, track_status, layout.x + layout.width - 125.0f * layout.scale,
                    layout.y + 10.0f * layout.scale, 12.0f * layout.scale, dev_col);

    // Bipolar center-zero bar for Cross-track error
    draw_bipolar_indicator(layout.x + 120.0f * layout.scale, layout.y + 27.0f * layout.scale,
                           layout.width - 132.0f * layout.scale, cte_mm / 40.0f, dev_col,
                           layout.scale);

    draw_hairline(layout.x + 12.0f * layout.scale, layout.y + 48.0f * layout.scale,
                  layout.width - 24.0f * layout.scale);

    // Lower row: Stanley Controller States
    draw_text_crisp(font_bold, "STANLEY TRAJECTORY CONTROLLER", layout.x + 12.0f * layout.scale,
                    layout.y + 54.0f * layout.scale, 12.0f * layout.scale, HudTheme::kTextMuted);

    const float col_step = (layout.width - 24.0f * layout.scale) / 3.0f;
    std::array<char, 32> hdg_str{};
    const float psi_deg = tel.heading_error_rad * constants::kRadToDeg;
    (void)snprintf(hdg_str.data(), hdg_str.size(), "ψ_err: %+4.1f°", psi_deg);
    draw_text_crisp(font_mono, hdg_str.data(), layout.x + 12.0f * layout.scale,
                    layout.y + 75.0f * layout.scale, 13.5f * layout.scale, HudTheme::kTextMain);

    std::array<char, 32> str_str{};
    (void)snprintf(str_str.data(), str_str.size(), "Steer δ: %+4.2frad", tel.stanley_steer_rad);
    draw_text_crisp(font_mono, str_str.data(), layout.x + 12.0f * layout.scale + col_step,
                    layout.y + 75.0f * layout.scale, 13.5f * layout.scale, HudTheme::kAccent);

    std::array<char, 32> vt_str{};
    (void)snprintf(vt_str.data(), vt_str.size(), "V_tgt: %.2fm/s", tel.target_velocity_mps);
    draw_text_crisp(font_mono, vt_str.data(), layout.x + 12.0f * layout.scale + 2.0f * col_step,
                    layout.y + 75.0f * layout.scale, 13.5f * layout.scale, HudTheme::kWarn);

    layout.advance(h);
    return;
  }

  // RUN 1 MAPPING: LINE-CENTRIC COCKPIT
  draw_text_crisp(font_bold, "LINE DEVIATION", layout.x + 12.0f * layout.scale,
                  layout.y + 8.0f * layout.scale, 12.0f * layout.scale, HudTheme::kTextMuted);

  std::array<char, 32> dev_str{};
  (void)snprintf(dev_str.data(), dev_str.size(), "%+.1f mm", tel.centroid_pos_mm);
  Color dev_col = HudTheme::kSuccess;
  if (tel.line_lost || std::abs(tel.centroid_pos_mm) > 15.0f) {
    dev_col = HudTheme::kAlert;
  } else if (std::abs(tel.centroid_pos_mm) > 6.0f) {
    dev_col = HudTheme::kWarn;
  }
  draw_text_crisp(font_mono, dev_str.data(), layout.x + 12.0f * layout.scale,
                  layout.y + 22.0f * layout.scale, 18.0f * layout.scale, dev_col);

  const char* track_status = "TRACK LOCKED";
  if (tel.bypass_state != 0) {
    track_status = "OBSTACLE BYPASS";
  } else if (tel.line_lost) {
    track_status = "LINE LOST";
  }
  const Color status_col = (tel.bypass_state != 0) ? HudTheme::kWarn : dev_col;
  draw_text_crisp(font_bold, track_status, layout.x + layout.width - 130.0f * layout.scale,
                  layout.y + 10.0f * layout.scale, 12.0f * layout.scale, status_col);

  // Bipolar center-zero bar
  draw_bipolar_indicator(layout.x + 120.0f * layout.scale, layout.y + 27.0f * layout.scale,
                         layout.width - 132.0f * layout.scale, tel.centroid_pos_mm / 35.0f, dev_col,
                         layout.scale);

  draw_hairline(layout.x + 12.0f * layout.scale, layout.y + 48.0f * layout.scale,
                layout.width - 24.0f * layout.scale);

  // Lower row: Optical Sensor Array
  draw_text_crisp(font_bold, "OPTICAL ARRAY", layout.x + 12.0f * layout.scale,
                  layout.y + 54.0f * layout.scale, 12.0f * layout.scale, HudTheme::kTextMuted);

  const float bar_margin = 12.0f * layout.scale;
  const float total_bars_w = layout.width - 2.0f * bar_margin;
  const float slot_w = total_bars_w / 8.0f;
  const float eye_w = slot_w - 6.0f * layout.scale;
  const float max_bar_h = 30.0f * layout.scale;

  for (size_t i = 0; i < lf::LineSensorConfig::kSensorCount; ++i) {
    const float bx = layout.x + bar_margin + static_cast<float>(i) * slot_w + 3.0f * layout.scale;
    const float b_val = in.ir_raw.at(i);
    const float b_h = std::clamp(b_val * max_bar_h, 2.0f * layout.scale, max_bar_h);

    DrawRectangle(static_cast<int>(bx), static_cast<int>(layout.y + 70.0f * layout.scale),
                  static_cast<int>(eye_w), static_cast<int>(max_bar_h), HudTheme::kSurfaceLight);
    const Color bar_col = (b_val > 0.60f) ? HudTheme::kSuccess : HudTheme::kAccent;
    DrawRectangle(static_cast<int>(bx),
                  static_cast<int>(layout.y + 70.0f * layout.scale + (max_bar_h - b_h)),
                  static_cast<int>(eye_w), static_cast<int>(b_h), bar_col);

    std::array<char, 8> lbl{};
    (void)snprintf(lbl.data(), lbl.size(), "S%zu", i);
    draw_text_crisp(font_mono, lbl.data(), bx + 1.0f * layout.scale,
                    layout.y + 101.0f * layout.scale, 11.0f * layout.scale, HudTheme::kTextDim);
  }

  layout.advance(h);
}

void draw_drivetrain_and_range_section(LayoutCursor& layout, float left_pwm, float right_pwm,
                                       const lf::SensorData& in, const lf::Telemetry& tel,
                                       size_t obstacle_count, float hovered_angle_deg,
                                       const Font& font_bold, const Font& font_mono) {
  const float h = 108.0f;
  draw_panel(Rectangle{layout.x, layout.y, layout.width, h * layout.scale});

  // Motors
  draw_text_crisp(font_bold, "DRIVETRAIN", layout.x + 12.0f * layout.scale,
                  layout.y + 8.0f * layout.scale, 12.0f * layout.scale, HudTheme::kTextMuted);

  const int l_rpm =
    static_cast<int>(std::round((tel.wheel_l.actual_speed_mps * 60.0f) / kCircumferenceM));
  const int r_rpm =
    static_cast<int>(std::round((tel.wheel_r.actual_speed_mps * 60.0f) / kCircumferenceM));

  std::array<char, 64> lp_str{};
  (void)snprintf(lp_str.data(), lp_str.size(), "L: %+3.0f%% · %.2fm/s · %d RPM", left_pwm * 100.0f,
                 tel.wheel_l.actual_speed_mps, l_rpm);
  draw_text_crisp(font_mono, lp_str.data(), layout.x + 12.0f * layout.scale,
                  layout.y + 24.0f * layout.scale, 12.5f * layout.scale, HudTheme::kTextMain);

  std::array<char, 64> rp_str{};
  (void)snprintf(rp_str.data(), rp_str.size(), "R: %+3.0f%% · %.2fm/s · %d RPM", right_pwm * 100.0f,
                 tel.wheel_r.actual_speed_mps, r_rpm);
  draw_text_crisp(font_mono, rp_str.data(), layout.x + 12.0f * layout.scale,
                  layout.y + 40.0f * layout.scale, 12.5f * layout.scale, HudTheme::kTextMain);

  const float bar_x = layout.x + layout.width - 150.0f * layout.scale;
  const float bar_w = 138.0f * layout.scale;
  DrawRectangle(static_cast<int>(bar_x), static_cast<int>(layout.y + 26.0f * layout.scale),
                static_cast<int>(bar_w), static_cast<int>(8.0f * layout.scale),
                HudTheme::kSurfaceLight);
  const float l_w = std::clamp(std::abs(left_pwm) * (bar_w * 0.5f), 0.0f, bar_w * 0.5f);
  DrawRectangle(static_cast<int>(bar_x + bar_w * 0.5f + ((left_pwm >= 0.0f) ? 0.0f : -l_w)),
                static_cast<int>(layout.y + 26.0f * layout.scale), static_cast<int>(l_w),
                static_cast<int>(8.0f * layout.scale),
                (left_pwm >= 0.0f) ? HudTheme::kAccent : HudTheme::kAlert);

  DrawRectangle(static_cast<int>(bar_x), static_cast<int>(layout.y + 42.0f * layout.scale),
                static_cast<int>(bar_w), static_cast<int>(8.0f * layout.scale),
                HudTheme::kSurfaceLight);
  const float r_w = std::clamp(std::abs(right_pwm) * (bar_w * 0.5f), 0.0f, bar_w * 0.5f);
  DrawRectangle(static_cast<int>(bar_x + bar_w * 0.5f + ((right_pwm >= 0.0f) ? 0.0f : -r_w)),
                static_cast<int>(layout.y + 42.0f * layout.scale), static_cast<int>(r_w),
                static_cast<int>(8.0f * layout.scale),
                (right_pwm >= 0.0f) ? HudTheme::kAccent : HudTheme::kAlert);

  draw_hairline(layout.x + 12.0f * layout.scale, layout.y + 58.0f * layout.scale,
                layout.width - 24.0f * layout.scale);

  // Rangefinders (ToF)
  draw_text_crisp(font_bold, "RANGEFINDERS", layout.x + 12.0f * layout.scale,
                  layout.y + 64.0f * layout.scale, 12.0f * layout.scale, HudTheme::kTextMuted);

  std::array<char, 32> obs_info{};
  if (hovered_angle_deg >= 0.0f) {
    (void)snprintf(obs_info.data(), obs_info.size(), "Angle: %.0f°", hovered_angle_deg);
  } else {
    (void)snprintf(obs_info.data(), obs_info.size(), "%zu Obstacles", obstacle_count);
  }
  draw_text_crisp(font_mono, obs_info.data(), layout.x + layout.width - 110.0f * layout.scale,
                  layout.y + 64.0f * layout.scale, 12.0f * layout.scale, HudTheme::kTextDim);

  const float col_step = (layout.width - 24.0f * layout.scale) / 3.0f;
  std::array<char, 24> l_tof{};
  (void)snprintf(l_tof.data(), l_tof.size(), "L: %.2f m", in.tof_left_m);
  draw_text_crisp(font_mono, l_tof.data(), layout.x + 12.0f * layout.scale,
                  layout.y + 82.0f * layout.scale, 14.0f * layout.scale, HudTheme::kTextMain);

  std::array<char, 24> f_tof{};
  (void)snprintf(f_tof.data(), f_tof.size(), "F: %.2f m", in.tof_front_m);
  const Color f_col = (in.tof_front_m < 0.15f) ? HudTheme::kAlert : HudTheme::kAccent;
  draw_text_crisp(font_mono, f_tof.data(), layout.x + 12.0f * layout.scale + col_step,
                  layout.y + 82.0f * layout.scale, 14.0f * layout.scale, f_col);

  std::array<char, 24> r_tof{};
  (void)snprintf(r_tof.data(), r_tof.size(), "R: %.2f m", in.tof_right_m);
  draw_text_crisp(font_mono, r_tof.data(), layout.x + 12.0f * layout.scale + 2.0f * col_step,
                  layout.y + 82.0f * layout.scale, 14.0f * layout.scale, HudTheme::kTextMain);

  layout.advance(h);
}

void draw_performance_section(LayoutCursor& layout, const SimRobotState& state,
                              float current_dist_m, const Font& font_bold, const Font& font_mono) {
  const float h = 92.0f;
  draw_panel(Rectangle{layout.x, layout.y, layout.width, h * layout.scale});

  draw_text_crisp(font_bold, "SPEED PROFILE", layout.x + 12.0f * layout.scale,
                  layout.y + 7.0f * layout.scale, 12.0f * layout.scale, HudTheme::kTextMuted);

  std::array<char, 48> r1_str{};
  if (state.has_run1_data || state.run1_time_s > 0.01f) {
    (void)snprintf(r1_str.data(), r1_str.size(), "Run 1: %05.2fs", state.run1_time_s);
  } else {
    (void)snprintf(r1_str.data(), r1_str.size(), "Run 1: --.--s");
  }
  draw_text_crisp(font_mono, r1_str.data(), layout.x + 12.0f * layout.scale,
                  layout.y + 23.0f * layout.scale, 13.0f * layout.scale, HudTheme::kAccent);

  std::array<char, 48> r2_str{};
  if (state.has_run2_data || state.run2_time_s > 0.01f) {
    const float delta_s = state.run2_time_s - state.run1_time_s;
    const float pct = (state.run1_time_s > 0.01f) ? ((delta_s / state.run1_time_s) * 100.0f) : 0.0f;
    (void)snprintf(r2_str.data(), r2_str.size(), "Run 2: %05.2fs (%+.1f%%)", state.run2_time_s,
                   pct);
  } else {
    (void)snprintf(r2_str.data(), r2_str.size(), "Run 2: --.--s");
  }
  draw_text_crisp(font_mono, r2_str.data(), layout.x + layout.width * 0.5f,
                  layout.y + 23.0f * layout.scale, 13.0f * layout.scale, HudTheme::kAccent);

  // Sparkline velocity graph
  const float spark_x = layout.x + 12.0f * layout.scale;
  const float spark_y = layout.y + 42.0f * layout.scale;
  const float spark_w = layout.width - 24.0f * layout.scale;
  const float spark_h = 42.0f * layout.scale;

  DrawRectangle(static_cast<int>(spark_x), static_cast<int>(spark_y), static_cast<int>(spark_w),
                static_cast<int>(spark_h), HudTheme::kSurfaceLight);
  DrawRectangleLines(static_cast<int>(spark_x), static_cast<int>(spark_y),
                     static_cast<int>(spark_w), static_cast<int>(spark_h), HudTheme::kBorder);

  if (state.planned_trajectory.size() > 5) {
    const size_t n = state.planned_trajectory.size();
    float max_traj_v = 0.5f;
    for (const auto& pt : state.planned_trajectory) {
      max_traj_v = std::max(max_traj_v, pt.target_speed_mps);
    }

    for (size_t i = 1; i < n; ++i) {
      const float x1 = spark_x + (static_cast<float>(i - 1) / static_cast<float>(n - 1)) * spark_w;
      const float y1 = spark_y + spark_h -
                       (state.planned_trajectory.at(i - 1).target_speed_mps / max_traj_v) *
                         (spark_h - 4.0f * layout.scale) -
                       2.0f * layout.scale;
      const float x2 = spark_x + (static_cast<float>(i) / static_cast<float>(n - 1)) * spark_w;
      const float y2 = spark_y + spark_h -
                       (state.planned_trajectory.at(i).target_speed_mps / max_traj_v) *
                         (spark_h - 4.0f * layout.scale) -
                       2.0f * layout.scale;
      DrawLineEx(Vector2{x1, y1}, Vector2{x2, y2}, 1.5f * layout.scale, HudTheme::kAccent);
    }

    // Live car cursor
    if (state.run1_time_s > 0.0f || state.has_run1_data) {
      const float est_total_dist = static_cast<float>(n) * 0.02f;
      const float cursor_ratio =
        std::clamp(current_dist_m / std::max(1.0f, est_total_dist), 0.0f, 1.0f);
      const float cursor_x = spark_x + cursor_ratio * spark_w;
      DrawLineEx(Vector2{cursor_x, spark_y}, Vector2{cursor_x, spark_y + spark_h},
                 2.0f * layout.scale, HudTheme::kSuccess);
    }
  } else {
    draw_text_crisp(font_mono, "Run 1 mapping needed to profile track",
                    spark_x + 16.0f * layout.scale, spark_y + 14.0f * layout.scale,
                    12.0f * layout.scale, HudTheme::kTextDim);
  }

  layout.advance(h);
}

void update_tab_navigation(int& focused_idx, int total_params) {
  if (!IsKeyPressed(KEY_TAB)) {
    return;
  }
  const bool reverse = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
  focused_idx = (focused_idx + (reverse ? (total_params - 1) : 1)) % total_params;
}

bool step_tuning_parameter(int focused_idx, int param_id, float& val, float min_v, float max_v,
                           float step, bool step_left, bool step_right) {
  if (focused_idx != param_id) {
    return false;
  }
  if (step_left) {
    val = std::clamp(val - step, min_v, max_v);
    return true;
  }
  if (step_right) {
    val = std::clamp(val + step, min_v, max_v);
    return true;
  }
  return false;
}

void draw_tuning_grid(LayoutCursor& layout, const Font& font_mono, int focused_idx, float cur_kp,
                      float cur_kd, float cur_v_map, float cur_obs_dist, float cur_obs_clear,
                      float cur_obs_len) {
  const float half_w = (layout.width - 24.0f * layout.scale) * 0.5f;
  float py = layout.y + 28.0f * layout.scale;

  const auto draw_param_cell = [&](int idx, const char* name, const char* hotkey, float val,
                                   const char* fmt, float x, float y) {
    const bool is_focused = (focused_idx == idx);
    std::array<char, 48> buf{};
    (void)snprintf(buf.data(), buf.size(), fmt, is_focused ? "> " : "  ", name, val, hotkey);
    draw_text_crisp(font_mono, buf.data(), x, y, 12.0f * layout.scale,
                    is_focused ? HudTheme::kBorderActive : HudTheme::kTextMain);
  };

  draw_param_cell(0, "Kp", "U/J", cur_kp, "%s%s: %.4f [%s]", layout.x + 12.0f * layout.scale, py);
  draw_param_cell(1, "Kd", "I/K", cur_kd, "%s%s: %.4f [%s]",
                  layout.x + 12.0f * layout.scale + half_w, py);

  py += 20.0f * layout.scale;
  draw_param_cell(2, "V_map", "E/D", cur_v_map, "%s%s: %.2fm/s [%s]",
                  layout.x + 12.0f * layout.scale, py);
  draw_param_cell(3, "D_obs", "Y/H", cur_obs_dist, "%s%s: %.2fm [%s]",
                  layout.x + 12.0f * layout.scale + half_w, py);

  py += 20.0f * layout.scale;
  draw_param_cell(4, "Clear", "B/G", cur_obs_clear, "%s%s: %.2fm [%s]",
                  layout.x + 12.0f * layout.scale, py);
  draw_param_cell(5, "L_chic", "N/V", cur_obs_len, "%s%s: %.2fm [%s]",
                  layout.x + 12.0f * layout.scale + half_w, py);
}

void draw_auto_tune_card(const LayoutCursor& layout, const lf::RobotCoordinator& robot,
                         const Font& font_bold, const Font& font_mono) {
  const float card_y = layout.y + 104.0f * layout.scale;
  const float card_w = layout.width - 24.0f * layout.scale;
  const float card_h = 68.0f * layout.scale;
  const Rectangle card_rect{layout.x + 12.0f * layout.scale, card_y, card_w, card_h};

  const auto robot_mode = robot.get_mode();
  if (robot_mode == lf::RobotMode::kAutoTune) {
    DrawRectangleRounded(card_rect, 0.15f, 4, ColorAlpha(HudTheme::kSuccess, 0.18f));
    DrawRectangleRoundedLines(card_rect, 0.15f, 4, HudTheme::kSuccess);

    draw_text_crisp(font_bold, "AUTO-TUNER ACTIVE [Åström–Hägglund Relay]",
                    card_rect.x + 10.0f * layout.scale, card_y + 8.0f * layout.scale,
                    12.0f * layout.scale, HudTheme::kSuccess);

    std::array<char, 64> t_buf{};
    (void)snprintf(t_buf.data(), t_buf.size(), "Cycles: %d / 4   |   Amp: %.1f mm",
                   robot.get_telemetry().tuner_cycles, robot.get_telemetry().tuner_amplitude_mm);
    draw_text_crisp(font_mono, t_buf.data(), card_rect.x + 10.0f * layout.scale,
                    card_y + 26.0f * layout.scale, 12.0f * layout.scale, HudTheme::kTextMain);

    draw_text_crisp(font_mono, "Measuring limit cycle: Tu & Ku -> Kp, Kd...",
                    card_rect.x + 10.0f * layout.scale, card_y + 44.0f * layout.scale,
                    11.0f * layout.scale, HudTheme::kTextDim);
    return;
  }

  DrawRectangleRounded(card_rect, 0.15f, 4, HudTheme::kSurfaceLight);
  DrawRectangleRoundedLines(card_rect, 0.15f, 4, HudTheme::kBorder);

  draw_text_crisp(font_bold, "[T] RELAY AUTO-TUNE (First Principles)",
                  card_rect.x + 10.0f * layout.scale, card_y + 8.0f * layout.scale,
                  12.0f * layout.scale, HudTheme::kAccent);

  draw_text_crisp(font_mono, "Press [T] to oscillate & find optimal gains",
                  card_rect.x + 10.0f * layout.scale, card_y + 26.0f * layout.scale,
                  11.5f * layout.scale, HudTheme::kTextMain);

  draw_text_crisp(font_mono, "Guarantees zero-magic-number calibration",
                  card_rect.x + 10.0f * layout.scale, card_y + 44.0f * layout.scale,
                  11.0f * layout.scale, HudTheme::kTextDim);
}

bool draw_navigation_tuning_section(LayoutCursor& layout, lf::RobotCoordinator& robot,
                                    SimRobotState& state, const Font& font_bold,
                                    const Font& font_mono) {
  const float h = 186.0f;
  draw_panel(Rectangle{layout.x, layout.y, layout.width, h * layout.scale});

  draw_text_crisp(font_bold, "PID & TACTICAL NAVIGATION", layout.x + 12.0f * layout.scale,
                  layout.y + 8.0f * layout.scale, 13.0f * layout.scale, HudTheme::kAccent);

  draw_text_crisp(font_mono, "[Tab] cycle  [<-/->] tune",
                  layout.x + layout.width - 170.0f * layout.scale, layout.y + 9.0f * layout.scale,
                  11.0f * layout.scale, HudTheme::kTextDim);

  float cur_kp = 0.0f;
  float cur_ki = 0.0f;
  float cur_kd = 0.0f;
  robot.get_line_pid_gains(cur_kp, cur_ki, cur_kd);

  float cur_v_map = robot.get_config().profiler.safe_map_speed_mps;
  const float cur_v_sprint = robot.get_config().profiler.max_straight_speed_mps;

  float cur_obs_dist = 0.0f;
  float cur_obs_clear = 0.0f;
  float cur_obs_len = 0.0f;
  robot.get_obstacle_parameters(cur_obs_dist, cur_obs_clear, cur_obs_len);

  constexpr int kTotalParams = 6;
  update_tab_navigation(state.focused_param_idx, kTotalParams);

  const bool step_left = IsKeyPressed(KEY_LEFT) || IsKeyPressed(KEY_LEFT_BRACKET);
  const bool step_right = IsKeyPressed(KEY_RIGHT) || IsKeyPressed(KEY_RIGHT_BRACKET);

  bool changed = false;

  if (step_tuning_parameter(state.focused_param_idx, 0, cur_kp, 0.005f, 0.150f, 0.005f, step_left,
                            step_right) ||
      step_tuning_parameter(state.focused_param_idx, 1, cur_kd, 0.0005f, 0.050f, 0.001f, step_left,
                            step_right)) {
    robot.set_line_pid_gains(cur_kp, cur_ki, cur_kd);
    changed = true;
  }

  if (step_tuning_parameter(state.focused_param_idx, 2, cur_v_map, 0.20f, 1.50f, 0.05f, step_left,
                            step_right)) {
    robot.set_tactical_speeds(cur_v_map, cur_v_sprint);
    changed = true;
  }

  if (step_tuning_parameter(state.focused_param_idx, 3, cur_obs_dist, 0.10f, 0.60f, 0.02f,
                            step_left, step_right) ||
      step_tuning_parameter(state.focused_param_idx, 4, cur_obs_clear, 0.08f, 0.35f, 0.01f,
                            step_left, step_right) ||
      step_tuning_parameter(state.focused_param_idx, 5, cur_obs_len, 0.30f, 1.50f, 0.05f, step_left,
                            step_right)) {
    robot.set_obstacle_parameters(cur_obs_dist, cur_obs_clear, cur_obs_len);
    changed = true;
  }

  draw_tuning_grid(layout, font_mono, state.focused_param_idx, cur_kp, cur_kd, cur_v_map,
                   cur_obs_dist, cur_obs_clear, cur_obs_len);

  draw_hairline(layout.x + 12.0f * layout.scale, layout.y + 96.0f * layout.scale,
                layout.width - 24.0f * layout.scale);

  draw_auto_tune_card(layout, robot, font_bold, font_mono);

  layout.advance(h);
  return changed;
}

GuiAction draw_controls_section(LayoutCursor& layout, bool follow_robot, Vector2 mouse_pos,
                                bool mouse_pressed, const SimRobotState& state,
                                const Font& font_bold, const Font& font_mono) {
  (void)follow_robot;
  const float h = 80.0f;
  draw_panel(Rectangle{layout.x, layout.y, layout.width, h * layout.scale});

  GuiAction action = GuiAction::kNone;

  const auto draw_btn = [&](Rectangle rect, const char* label, Color accent, GuiAction act,
                            int key_code) {
    const bool is_hovered = CheckCollisionPointRec(mouse_pos, rect);
    const bool is_flashing = (state.key_flash_timer > 0.0f && state.last_pressed_key == key_code);
    Color const bg =
      is_hovered || is_flashing ? ColorAlpha(accent, 0.25f) : HudTheme::kSurfaceLight;
    Color const border = is_hovered || is_flashing ? accent : HudTheme::kBorder;
    DrawRectangleRounded(rect, 0.25f, 4, bg);
    DrawRectangleRoundedLines(rect, 0.25f, 4, border);
    draw_text_crisp(font_bold, label, rect.x + 8.0f * layout.scale, rect.y + 7.0f * layout.scale,
                    12.5f * layout.scale,
                    (is_hovered || is_flashing) ? HudTheme::kTextMain : accent);
    if (is_hovered && mouse_pressed) {
      action = act;
    }
  };

  const float btn_w = (layout.width - 24.0f * layout.scale - 24.0f * layout.scale) / 5.0f;
  const float btn_h = 28.0f * layout.scale;
  float bx = layout.x + 12.0f * layout.scale;
  const float by = layout.y + 10.0f * layout.scale;

  draw_btn(Rectangle{bx, by, btn_w, btn_h}, "[T] TUNE", HudTheme::kSuccess, GuiAction::kAutoTune,
           KEY_T);
  bx += btn_w + 6.0f * layout.scale;
  draw_btn(Rectangle{bx, by, btn_w, btn_h}, "[1] MAP", HudTheme::kAccent, GuiAction::kRun1Map,
           KEY_ONE);
  bx += btn_w + 6.0f * layout.scale;
  draw_btn(Rectangle{bx, by, btn_w, btn_h}, "[2] SPRINT", HudTheme::kWarn, GuiAction::kRun2Sprint,
           KEY_TWO);
  bx += btn_w + 6.0f * layout.scale;
  draw_btn(Rectangle{bx, by, btn_w, btn_h}, "[SPC] PAUSE", HudTheme::kTextMain,
           GuiAction::kTogglePause, KEY_SPACE);
  bx += btn_w + 6.0f * layout.scale;
  draw_btn(Rectangle{bx, by, btn_w, btn_h}, "[R] RESET", HudTheme::kAlert, GuiAction::kResetPose,
           KEY_R);

  // Hotkey Matrix Cheat Sheet
  const float ky = layout.y + 48.0f * layout.scale;
  const auto draw_key_chip = [&](float x, const char* text, int key_code) {
    const bool is_flashing = (state.key_flash_timer > 0.0f && state.last_pressed_key == key_code);
    draw_text_crisp(font_mono, text, x, ky, 11.5f * layout.scale,
                    is_flashing ? HudTheme::kBorderActive : HudTheme::kTextDim);
  };

  float cx = layout.x + 12.0f * layout.scale;
  draw_key_chip(cx, "[C] Cam", KEY_C);
  cx += 56.0f * layout.scale;
  draw_key_chip(cx, "[F] Fit", KEY_F);
  cx += 52.0f * layout.scale;
  draw_key_chip(cx, "[X] Clear", KEY_X);
  cx += 62.0f * layout.scale;
  draw_key_chip(cx, "[Tab] Focus", KEY_TAB);
  cx += 78.0f * layout.scale;
  draw_key_chip(cx, "[<-/->] Step", KEY_LEFT);

  layout.advance(h);
  return action;
}

void draw_toast_banner(LayoutCursor& layout, const char* toast_msg, const Font& font_bold) {
  const float h = 28.0f;
  const Rectangle rect{layout.x, layout.y, layout.width, h * layout.scale};
  DrawRectangleRounded(rect, 0.20f, 4, ColorAlpha(HudTheme::kSurfaceLight, 0.95f));
  DrawRectangleRoundedLines(rect, 0.20f, 4, HudTheme::kAccent);
  draw_text_crisp(font_bold, toast_msg, layout.x + 12.0f * layout.scale,
                  layout.y + 7.0f * layout.scale, 12.0f * layout.scale, HudTheme::kTextMain);
  layout.advance(h);
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

HudResult draw_hud_sidebar(float sb_x, float sb_w, float view_h, float ui_scale,
                           lf::RobotCoordinator& robot, const lf::SensorData& in,
                           SimRobotState& state, size_t obstacle_count,
                           float hovered_obstacle_angle, Vector2 mouse_pos, bool mouse_down,
                           bool mouse_pressed, bool follow_robot, const char* toast_msg,
                           float toast_timer, const Font& font_bold, const Font& font_mono) {
  (void)mouse_down;
  // Draw background sidebar strip
  DrawRectangle(static_cast<int>(sb_x), 0, static_cast<int>(sb_w), static_cast<int>(view_h),
                HudTheme::kBg);
  DrawLineEx(Vector2{sb_x, 0}, Vector2{sb_x, view_h}, 1.5f * ui_scale, HudTheme::kBorder);

  const float pad_x = 16.0f * ui_scale;
  LayoutCursor layout{sb_x + pad_x, 12.0f * ui_scale, sb_w - 2.0f * pad_x, ui_scale};

  HudResult result{};

  draw_header_section(layout, state, robot.get_mode(), font_bold, font_mono);

  draw_hero_telemetry_section(layout, state.v, robot.get_config().profiler.max_straight_speed_mps,
                              state.longitudinal_accel_mps2, state.lateral_accel_mps2,
                              robot.get_config().profiler.corner_grip_mu, robot.get_telemetry(),
                              font_bold, font_mono);

  draw_tracking_and_sensors_section(layout, robot.get_telemetry(), in, font_bold, font_mono);

  draw_drivetrain_and_range_section(layout, state.left_pwm, state.right_pwm, in,
                                    robot.get_telemetry(), obstacle_count, hovered_obstacle_angle,
                                    font_bold, font_mono);

  draw_performance_section(layout, state, robot.get_telemetry().total_distance_m, font_bold,
                           font_mono);

  result.tactical_changed =
    draw_navigation_tuning_section(layout, robot, state, font_bold, font_mono);

  result.action = draw_controls_section(layout, follow_robot, mouse_pos, mouse_pressed, state,
                                        font_bold, font_mono);

  if (toast_timer > 0.0f && toast_msg != nullptr && toast_msg[0] != '\0') {
    draw_toast_banner(layout, toast_msg, font_bold);
  }

  return result;
}

void draw_canvas_hover_telemetry(Vector2 mouse_screen, const lf::TrajectoryPoint& pt, size_t pt_idx,
                                 float ui_scale, const Font& font_bold, const Font& font_mono) {
  const float tw = 250.0f * ui_scale;
  const float th = 48.0f * ui_scale;
  const float tx = mouse_screen.x + 15.0f * ui_scale;
  const float ty = mouse_screen.y + 15.0f * ui_scale;

  draw_panel(Rectangle{tx, ty, tw, th});
  DrawRectangleRoundedLines(Rectangle{tx, ty, tw, th}, 0.08f, 4, HudTheme::kAccent);

  std::array<char, 64> line1{};
  (void)snprintf(line1.data(), line1.size(), "WAYPOINT #%zu · s = %.2f m", pt_idx,
                 static_cast<float>(pt_idx) * 0.02f);
  draw_text_crisp(font_bold, line1.data(), tx + 10.0f * ui_scale, ty + 7.0f * ui_scale,
                  12.0f * ui_scale, HudTheme::kAccent);

  const float r_eff = (pt.curvature_rad_m > 0.01f) ? (1.0f / pt.curvature_rad_m) : 99.0f;
  const float a_lat_g = (pt.target_speed_mps * pt.target_speed_mps * pt.curvature_rad_m) / 9.81f;

  std::array<char, 80> line2{};
  (void)snprintf(line2.data(), line2.size(), "V: %.2fm/s · R: %.0fmm · %.2fG", pt.target_speed_mps,
                 r_eff * 1000.0f, a_lat_g);
  draw_text_crisp(font_mono, line2.data(), tx + 10.0f * ui_scale, ty + 25.0f * ui_scale,
                  12.5f * ui_scale, HudTheme::kTextMain);
}

}  // namespace lf::sim
