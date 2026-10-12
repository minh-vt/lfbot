#include "renderer.hpp"

#include <raylib.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

#include "sensing/normalize.hpp"
#include "sim_config.hpp"

namespace lf::sim {
namespace {

constexpr int kDefaultWinW = 1920;
constexpr int kDefaultWinH = 1080;

// Bảng màu Neutral công nghiệp (Hallmark & Design-Taste)
constexpr Color kBgCharcoal = {14, 16, 19, 255};      // #0e1013
constexpr Color kBorderGray = {39, 39, 42, 255};      // #27272a
constexpr Color kTextMuted = {161, 161, 170, 255};    // #a1a1aa
constexpr Color kTextPrimary = {244, 244, 245, 255};  // #f4f4f5
constexpr Color kZincWhite = {250, 250, 250, 255};
constexpr Color kAmberAlert = {245, 158, 11, 255};  // #f59e0b

constexpr float kTopBarBaseH = 64.0f;
constexpr float kDockBaseH = 76.0f;

const char* run_mode_name(int mode) {
  switch (mode) {
    case 0:
      return "IDLE";
    case 1:
      return "RUN 1 (MAP)";
    case 2:
      return "RUN 2 (SPRINT)";
    default:
      return "DONE";
  }
}

void draw_text_bold(const char* text, int x, int y, int size, Color color) {
  DrawText(text, x, y, size, color);
  DrawText(text, x + 1, y, size, color);
}

int scaled_font_size(int base_size, float ui_scale) {
  return std::max(32, static_cast<int>(static_cast<float>(base_size) * ui_scale));
}

}  // namespace

void Renderer::init(const TrackManifest& manifest) {
  SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT);
  InitWindow(kDefaultWinW, kDefaultWinH, "LF-Bot 2026 Autonomous Racing Platform");
  SetWindowMinSize(1280, 720);
  SetTargetFPS(60);
  reload_track(manifest);
}

void Renderer::reload_track(const TrackManifest& manifest) {
  if (texture_.id != 0) {
    UnloadTexture(texture_);
  }
  const Image image = LoadImage(manifest.image_path.c_str());
  texture_ = LoadTextureFromImage(image);
  UnloadImage(image);

  const auto screen_w = static_cast<float>(GetScreenWidth());
  const auto screen_h = static_cast<float>(GetScreenHeight());
  const float scale_y = screen_h / 1080.0f;
  const float ui_scale = std::max(1.0f, scale_y);
  const float dock_total_h = (kTopBarBaseH + kDockBaseH) * ui_scale + 40.0f;

  overview_scale_ = std::min((screen_w - 80.0f) / manifest.arena_width_mm,
                             (screen_h - dock_total_h) / manifest.arena_height_mm);

  smooth_target_ = Vector2{manifest.start_x_mm, manifest.start_y_mm};
  camera_.target = smooth_target_;
  camera_.offset = Vector2{screen_w * 0.5f, screen_h * 0.5f};
  camera_.rotation = 0.0f;
  camera_.zoom = chase_camera_ ? 1.6f : overview_scale_;
}

void Renderer::shutdown() {
  if (texture_.id != 0) {
    UnloadTexture(texture_);
  }
  CloseWindow();
}

bool Renderer::should_close() {
  return WindowShouldClose();
}

void Renderer::begin_frame() {
  BeginDrawing();
}

void Renderer::end_frame() {
  EndDrawing();
}

void Renderer::toggle_camera_mode() {
  chase_camera_ = !chase_camera_;
}

void Renderer::update_camera(float rx, float ry, float theta, float v, float dt,
                             const TrackManifest& manifest) {
  const auto screen_w = static_cast<float>(GetScreenWidth());
  const auto screen_h = static_cast<float>(GetScreenHeight());
  const float scale_y = screen_h / 1080.0f;
  const float ui_scale = std::max(1.0f, scale_y);
  const float dock_total_h = (kTopBarBaseH + kDockBaseH) * ui_scale + 40.0f;

  overview_scale_ = std::min((screen_w - 80.0f) / manifest.arena_width_mm,
                             (screen_h - dock_total_h) / manifest.arena_height_mm);

  if (chase_camera_) {
    const float lookahead = std::max(v * 120.0f, 60.0f);
    const float target_x = rx + std::cos(theta) * lookahead;
    const float target_y = ry + std::sin(theta) * lookahead;

    const float alpha = std::clamp(dt * 7.0f, 0.0f, 1.0f);
    smooth_target_.x += (target_x - smooth_target_.x) * alpha;
    smooth_target_.y += (target_y - smooth_target_.y) * alpha;

    camera_.target = smooth_target_;
    camera_.offset = Vector2{screen_w * 0.5f, screen_h * 0.5f};
    camera_.zoom = 1.6f;
  } else {
    camera_.target = Vector2{manifest.arena_width_mm * 0.5f, manifest.arena_height_mm * 0.5f};
    camera_.offset = Vector2{screen_w * 0.5f, screen_h * 0.5f};
    camera_.zoom = overview_scale_;
  }
}

void Renderer::draw_ghost_chassis(float rx, float ry, float theta) {
  const float cos_t = std::cos(theta);
  const float sin_t = std::sin(theta);
  const float hl = kRobotHalfLengthMm;
  const float hw = kRobotHalfWidthMm;

  const Vector2 p1 = {rx + hl * cos_t - hw * sin_t, ry + hl * sin_t + hw * cos_t};
  const Vector2 p2 = {rx + hl * cos_t + hw * sin_t, ry + hl * sin_t - hw * cos_t};
  const Vector2 p3 = {rx - hl * cos_t + hw * sin_t, ry - hl * sin_t - hw * cos_t};
  const Vector2 p4 = {rx - hl * cos_t - hw * sin_t, ry - hl * sin_t + hw * cos_t};

  DrawRectanglePro(Rectangle{rx, ry, hl * 2.0f, hw * 2.0f}, Vector2{hl, hw}, theta * 180.0f / kPi,
                   Color{39, 39, 42, 60});
  DrawLineEx(p1, p2, 1.8f, Color{113, 113, 122, 100});
  DrawLineEx(p2, p3, 1.8f, Color{113, 113, 122, 100});
  DrawLineEx(p3, p4, 1.8f, Color{113, 113, 122, 100});
  DrawLineEx(p4, p1, 1.8f, Color{113, 113, 122, 100});
}

void Renderer::draw_chassis(float rx, float ry, float theta, float path_steer,
                            const std::array<float, 8>& ir_weights, float d_front, float d_left,
                            float d_right) {
  const float cos_t = std::cos(theta);
  const float sin_t = std::sin(theta);
  const float front_x = rx + kSensorLookaheadMm * cos_t;
  const float front_y = ry + kSensorLookaheadMm * sin_t;
  const float perp_x = -sin_t;
  const float perp_y = cos_t;

  const float hl = kRobotHalfLengthMm;
  const float hw = kRobotHalfWidthMm;

  // 3 Nón ToF: 1 ở phía trước, 2 ở hai bên sườn xe, mang 3 màu sắc khác biệt.
  const auto draw_cone = [](Vector2 origin, float center_rad, float dist_mm, Color col) {
    constexpr float kHalfFovDeg = 12.5f;
    const float center_deg = center_rad * 180.0f / kPi;
    const float start_deg = center_deg - kHalfFovDeg;
    const float end_deg = center_deg + kHalfFovDeg;
    const float r = std::clamp(dist_mm, 20.0f, sensing::kTofRangeMm);
    const bool danger = (dist_mm < 280.0f);

    const Color fill = danger ? Color{245, 158, 11, 50} : Color{col.r, col.g, col.b, 35};
    const Color edge = danger ? Color{245, 158, 11, 230} : Color{col.r, col.g, col.b, 200};

    DrawCircleSector(origin, r, start_deg, end_deg, 14, fill);
    DrawCircleSectorLines(origin, r, start_deg, end_deg, 14, edge);
    DrawCircleV(origin, 3.2f, danger ? kAmberAlert : col);
  };

  // 1. Front ToF (Mũi trước, 0°): Sky Blue #38bdf8
  const Vector2 pos_front = {rx + hl * cos_t, ry + hl * sin_t};
  constexpr Color kToFFrontCol = {56, 189, 248, 255};
  draw_cone(pos_front, theta, d_front, kToFFrontCol);

  // 2. Left ToF (Sườn trái, -26°): Emerald Green #34d399
  const Vector2 pos_left = {rx + 15.0f * cos_t - hw * perp_x, ry + 15.0f * sin_t - hw * perp_y};
  constexpr Color kToFLeftCol = {52, 211, 153, 255};
  draw_cone(pos_left, theta - 0.45f, d_left, kToFLeftCol);

  // 3. Right ToF (Sườn phải, +26°): Violet #a855f7
  const Vector2 pos_right = {rx + 15.0f * cos_t + hw * perp_x, ry + 15.0f * sin_t + hw * perp_y};
  constexpr Color kToFRightCol = {168, 85, 247, 255};
  draw_cone(pos_right, theta + 0.45f, d_right, kToFRightCol);

  // Khung xe 145x138mm
  const Vector2 p1 = {rx + hl * cos_t - hw * sin_t, ry + hl * sin_t + hw * cos_t};
  const Vector2 p2 = {rx + hl * cos_t + hw * sin_t, ry + hl * sin_t - hw * cos_t};
  const Vector2 p3 = {rx - hl * cos_t + hw * sin_t, ry - hl * sin_t - hw * cos_t};
  const Vector2 p4 = {rx - hl * cos_t - hw * sin_t, ry - hl * sin_t + hw * cos_t};

  DrawRectanglePro(Rectangle{rx, ry, hl * 2.0f, hw * 2.0f}, Vector2{hl, hw}, theta * 180.0f / kPi,
                   Color{24, 24, 27, 240});
  DrawLineEx(p1, p2, 1.8f, Color{244, 244, 245, 230});
  DrawLineEx(p2, p3, 1.8f, Color{244, 244, 245, 230});
  DrawLineEx(p3, p4, 1.8f, Color{244, 244, 245, 230});
  DrawLineEx(p4, p1, 1.8f, Color{244, 244, 245, 230});

  // 8 Mắt IR micro-LED trên cản trước
  for (std::size_t i = 0; i < sensing::kIrOffsetsMm.size(); ++i) {
    const float off = sensing::kIrOffsetsMm[i];
    const float sx = front_x + off * perp_x;
    const float sy = front_y + off * perp_y;
    const float weight = ir_weights[i];
    if (weight > 0.1f) {
      DrawCircleV(Vector2{sx, sy}, 3.5f, kZincWhite);
      DrawCircleLines(static_cast<int>(sx), static_cast<int>(sy), 4.5f, Color{250, 250, 250, 100});
    } else {
      DrawCircleV(Vector2{sx, sy}, 2.2f, Color{63, 63, 70, 180});
    }
  }

  // Véc-tơ lái path_steer
  const float steer_angle = theta + path_steer * 0.45f;
  constexpr float kNeedleLen = 42.0f;
  const Vector2 needle_tip = {
    front_x + kNeedleLen * std::cos(steer_angle),
    front_y + kNeedleLen * std::sin(steer_angle),
  };
  DrawLineEx(Vector2{front_x, front_y}, needle_tip, 2.0f, Color{250, 250, 250, 220});
  DrawCircleV(needle_tip, 2.5f, Color{250, 250, 250, 220});
}

void Renderer::draw_trail_run1(const std::vector<TracePoint>& trace,
                               const lf::drive::RacePlan& plan) {
  if (trace.size() < 2) {
    return;
  }
  const bool has_plan = plan.ready;
  constexpr float kMinDistSq = 15.0f * 15.0f;

  std::size_t prev_idx = 0;
  for (std::size_t i = 1; i < trace.size(); ++i) {
    const float dx = trace[i].x - trace[prev_idx].x;
    const float dy = trace[i].y - trace[prev_idx].y;
    const bool is_last = (i == trace.size() - 1);
    if (dx * dx + dy * dy < kMinDistSq && !is_last) {
      continue;
    }
    const Vector2 p1 = {trace[prev_idx].x, trace[prev_idx].y};
    const Vector2 p2 = {trace[i].x, trace[i].y};

    if (has_plan) {
      // Heatmap Ribbon: Tốc độ cao = trắng, giảm tốc backward-pass = xám tối
      const float speed = std::clamp(trace[prev_idx].speed_limit, 0.2f, 1.0f);
      const auto intensity = static_cast<unsigned char>(60.0f + (speed - 0.2f) * (190.0f / 0.8f));
      const Color col = {intensity, intensity, intensity, 220};
      DrawLineEx(p1, p2, 3.2f, col);

      if (std::abs(trace[prev_idx].obstacle_side) > 0.05f) {
        DrawCircleV(p1, 2.0f, Color{245, 158, 11, 140});
      }
    } else {
      DrawLineEx(p1, p2, 2.0f, Color{113, 113, 122, 110});
    }
    prev_idx = i;
  }
}

void Renderer::draw_trail_run2(const std::vector<TracePoint>& trace) {
  if (trace.size() < 2) {
    return;
  }
  constexpr float kMinDistSq = 15.0f * 15.0f;
  std::size_t prev_idx = 0;
  for (std::size_t i = 1; i < trace.size(); ++i) {
    const float dx = trace[i].x - trace[prev_idx].x;
    const float dy = trace[i].y - trace[prev_idx].y;
    const bool is_last = (i == trace.size() - 1);
    if (dx * dx + dy * dy < kMinDistSq && !is_last) {
      continue;
    }
    const Vector2 p1 = {trace[prev_idx].x, trace[prev_idx].y};
    const Vector2 p2 = {trace[i].x, trace[i].y};
    DrawLineEx(p1, p2, 2.5f, Color{250, 250, 250, 230});
    prev_idx = i;
  }
}

void Renderer::draw_world(const TrackManifest& manifest, const RenderFrame& frame,
                          const lf::drive::RacePlan& plan,
                          const std::vector<TracePoint>& run1_trace,
                          const std::vector<TracePoint>& run2_trace) {
  DrawTexture(texture_, 0, 0, WHITE);

  DrawRectangleLinesEx(Rectangle{0.0f, 0.0f, manifest.arena_width_mm, manifest.arena_height_mm},
                       2.5f, kBorderGray);

  // Vạch đích
  const Rectangle finish_rec{
    manifest.finish_min_x,
    manifest.finish_min_y,
    manifest.finish_max_x - manifest.finish_min_x,
    manifest.finish_max_y - manifest.finish_min_y,
  };
  DrawRectangleRec(finish_rec, Color{244, 244, 245, 25});
  DrawRectangleLinesEx(finish_rec, 2.0f, Color{244, 244, 245, 180});

  // Vật cản
  for (const auto& obs : manifest.obstacles) {
    const Rectangle obs_rec{
      obs.x_mm - obs.width_mm * 0.5f,
      obs.y_mm - obs.height_mm * 0.5f,
      obs.width_mm,
      obs.height_mm,
    };
    DrawRectangleRec(obs_rec, Color{39, 39, 42, 220});
    DrawRectangleLinesEx(obs_rec, 2.0f, kAmberAlert);
  }

  // Quỹ đạo Run 1 & Run 2
  draw_trail_run1(run1_trace, plan);
  draw_trail_run2(run2_trace);

  // Ghost Car Run 1 (khi đang ở Run 2 hoặc replay)
  if (frame.mode == 2 && !run1_trace.empty()) {
    const auto it =
      std::lower_bound(run1_trace.begin(), run1_trace.end(), frame.elapsed_s,
                       [](const TracePoint& pt, float val) { return pt.elapsed_s < val; });
    if (it != run1_trace.end()) {
      draw_ghost_chassis(it->x, it->y, it->theta);
    }
  }

  // Xe chính
  draw_chassis(frame.x, frame.y, frame.theta, frame.path_steer, frame.ir_weights, frame.tof_front,
               frame.tof_left, frame.tof_right);
}

void Renderer::draw_top_dock(const TrackManifest& manifest, const RenderFrame& frame) const {
  const auto screen_w = static_cast<float>(GetScreenWidth());
  const auto screen_h = static_cast<float>(GetScreenHeight());
  const float scale_x = screen_w / 1920.0f;
  const float scale_y = screen_h / 1080.0f;
  const float ui_scale = std::max(1.0f, std::min(scale_x, scale_y));

  const float top_bar_h = std::max(kTopBarBaseH, kTopBarBaseH * ui_scale);
  const int font_sz = scaled_font_size(32, ui_scale);
  const int text_y = static_cast<int>((top_bar_h - static_cast<float>(font_sz)) * 0.5f);

  DrawRectangle(0, 0, static_cast<int>(screen_w), static_cast<int>(top_bar_h),
                Color{14, 16, 19, 245});
  DrawLine(0, static_cast<int>(top_bar_h), static_cast<int>(screen_w), static_cast<int>(top_bar_h),
           kBorderGray);

  std::array<char, 128> text_buf{};
  (void)std::snprintf(text_buf.data(), text_buf.size(), "TRACK: %s", manifest.name.c_str());
  draw_text_bold(text_buf.data(), 24, text_y, font_sz, kZincWhite);

  Color mode_color = kBorderGray;
  if (frame.mode == 2) {
    mode_color = kZincWhite;
  } else if (frame.mode == 1) {
    mode_color = kTextMuted;
  }
  const int mode_x = static_cast<int>(screen_w * 0.20f);
  (void)std::snprintf(text_buf.data(), text_buf.size(), "MODE: %s", run_mode_name(frame.mode));
  draw_text_bold(text_buf.data(), mode_x, text_y, font_sz, mode_color);

  const int tel_x = static_cast<int>(screen_w * 0.40f);
  (void)std::snprintf(text_buf.data(), text_buf.size(), "t: %.2fs | v: %.2fm/s | cert: %.0f%%",
                      frame.elapsed_s, frame.v, frame.line_certainty * 100.0f);
  draw_text_bold(text_buf.data(), tel_x, text_y, font_sz, kTextPrimary);

  const char* cam_name = chase_camera_ ? "CHASE CAM" : "OVERVIEW";
  const char* play_state = frame.paused ? "PAUSED" : "RUNNING";
  (void)std::snprintf(text_buf.data(), text_buf.size(), "%s | %.2fx | %s", cam_name,
                      frame.playback_speed, play_state);
  const int cam_w = MeasureText(text_buf.data(), font_sz);
  const int cam_x = static_cast<int>(screen_w) - cam_w - 24;
  draw_text_bold(text_buf.data(), cam_x, text_y, font_sz, frame.paused ? kAmberAlert : kTextMuted);
}

void Renderer::draw_telemetry_dock(const RenderFrame& frame) {
  const auto screen_w = static_cast<float>(GetScreenWidth());
  const auto screen_h = static_cast<float>(GetScreenHeight());
  const float scale_x = screen_w / 1920.0f;
  const float scale_y = screen_h / 1080.0f;
  const float ui_scale = std::max(1.0f, std::min(scale_x, scale_y));

  const float dock_h = std::max(kDockBaseH, kDockBaseH * ui_scale);
  const float dock_y = screen_h - dock_h;
  const int font_sz = scaled_font_size(32, ui_scale);
  const int text_y = static_cast<int>(dock_y + (dock_h - static_cast<float>(font_sz)) * 0.5f);

  DrawRectangle(0, static_cast<int>(dock_y), static_cast<int>(screen_w), static_cast<int>(dock_h),
                Color{14, 16, 19, 255});
  DrawLine(0, static_cast<int>(dock_y), static_cast<int>(screen_w), static_cast<int>(dock_y),
           kBorderGray);

  std::array<char, 128> text_buf{};

  // 1. Laps Summary (Font >= 32)
  const float delta_t =
    (frame.lap1_s > 0.0f && frame.lap2_s > 0.0f) ? (frame.lap2_s - frame.lap1_s) : 0.0f;
  (void)std::snprintf(text_buf.data(), text_buf.size(), "R1: %.2fs  R2: %.2fs  Δt: %+.2fs",
                      frame.lap1_s, frame.lap2_s, delta_t);
  draw_text_bold(text_buf.data(), 24, text_y, font_sz, kZincWhite);

  // 2. Motor PWM Meters: Nhãn rõ ràng, hiển thị phần trăm và thanh bar
  const int duty_base_x = static_cast<int>(screen_w * 0.28f);
  draw_text_bold("MOTOR:", duty_base_x, text_y, font_sz, kTextMuted);

  const auto draw_motor_bar = [font_sz, ui_scale](int x, int y, const char* label, float duty) {
    draw_text_bold(label, x, y, font_sz, kZincWhite);
    std::array<char, 16> pct_buf{};
    const int pct = static_cast<int>(duty * 100.0f);
    (void)std::snprintf(pct_buf.data(), pct_buf.size(), "%+3d%%", pct);
    const int pct_x = x + static_cast<int>(24.0f * ui_scale);
    draw_text_bold(pct_buf.data(), pct_x, y, font_sz, kTextPrimary);

    const int meter_w = static_cast<int>(70.0f * ui_scale);
    const int meter_h = static_cast<int>(18.0f * ui_scale);
    const int bx = pct_x + static_cast<int>(90.0f * ui_scale);
    const int by =
      y + static_cast<int>((static_cast<float>(font_sz) - static_cast<float>(meter_h)) * 0.5f);

    DrawRectangle(bx, by, meter_w, meter_h, Color{39, 39, 42, 255});
    DrawLine(bx + meter_w / 2, by - 2, bx + meter_w / 2, by + meter_h + 2, kBorderGray);
    const float clamped = std::clamp(duty, -1.0f, 1.0f);
    const int bar_len = static_cast<int>(std::abs(clamped) * (static_cast<float>(meter_w) / 2.0f));
    if (clamped >= 0.0f) {
      DrawRectangle(bx + meter_w / 2, by, bar_len, meter_h, kZincWhite);
    } else {
      DrawRectangle(bx + meter_w / 2 - bar_len, by, bar_len, meter_h, kAmberAlert);
    }
  };

  const int bar_offset_l = static_cast<int>(140.0f * ui_scale);
  const int bar_offset_r = static_cast<int>(340.0f * ui_scale);
  draw_motor_bar(duty_base_x + bar_offset_l, text_y, "L", frame.left_duty);
  draw_motor_bar(duty_base_x + bar_offset_r, text_y, "R", frame.right_duty);

  // 3. Line Offset & Alert Badge (Font >= 32)
  const int status_x = static_cast<int>(screen_w * 0.55f);
  (void)std::snprintf(text_buf.data(), text_buf.size(), "Offset: %+.2f", frame.line_offset);
  draw_text_bold(text_buf.data(), status_x, text_y, font_sz, kTextPrimary);

  const int badge_x = status_x + static_cast<int>(200.0f * ui_scale);
  if (frame.dq) {
    if (frame.dq_reason != nullptr) {
      (void)std::snprintf(text_buf.data(), text_buf.size(), "[ DQ: %s ]", frame.dq_reason);
      draw_text_bold(text_buf.data(), badge_x, text_y, font_sz, kAmberAlert);
    } else {
      draw_text_bold("[ DQ: INFRACTION ]", badge_x, text_y, font_sz, kAmberAlert);
    }
  } else if (frame.line_certainty < 0.25f) {
    draw_text_bold("[ LINE UNCERTAIN ]", badge_x, text_y, font_sz, kAmberAlert);
  } else {
    draw_text_bold("[ TRACKING ]", badge_x, text_y, font_sz, Color{161, 161, 170, 255});
  }

  // 4. Playback / Status info (Right aligned, Font >= 32)
  std::array<char, 64> status_buf{};
  Color status_col = kTextMuted;
  if (frame.scrub_frame >= 0) {
    (void)std::snprintf(status_buf.data(), status_buf.size(), "FRAME %d / %d",
                        frame.scrub_frame + 1, frame.total_frames);
    status_col = kAmberAlert;
  } else if (frame.dq) {
    (void)std::snprintf(status_buf.data(), status_buf.size(), "STOPPED (R: reset)");
    status_col = kAmberAlert;
  } else if (frame.paused) {
    (void)std::snprintf(status_buf.data(), status_buf.size(), "PAUSED (← / →)");
    status_col = kAmberAlert;
  } else {
    (void)std::snprintf(status_buf.data(), status_buf.size(), "LIVE (500Hz)");
    status_col = kTextMuted;
  }
  const int status_w = MeasureText(status_buf.data(), font_sz);
  const int info_x = static_cast<int>(screen_w) - status_w - 24;
  draw_text_bold(status_buf.data(), info_x, text_y, font_sz, status_col);
}

void Renderer::draw_hud(const TrackManifest& manifest, const RenderFrame& frame) const {
  draw_top_dock(manifest, frame);
  draw_telemetry_dock(frame);
}

void Renderer::draw(const TrackManifest& manifest, const RenderFrame& frame,
                    const lf::drive::RacePlan& plan, const std::vector<TracePoint>& run1_trace,
                    const std::vector<TracePoint>& run2_trace) {
  ClearBackground(kBgCharcoal);

  BeginMode2D(camera_);
  draw_world(manifest, frame, plan, run1_trace, run2_trace);
  EndMode2D();

  draw_hud(manifest, frame);
}

}  // namespace lf::sim
