#include "renderer.hpp"

#include <raylib.h>

#include <algorithm>
#include <array>
#include <cmath>

#include "robot_geometry.hpp"

namespace lf::sim {
namespace {

// Layout robot hiện tại (SSoT) — renderer đọc hình học từ đây.
constexpr RobotGeometry kLayout = kSimulatedRobot;
constexpr float kPi = 3.14159265358979323846F;

// 4 góc khung xe tại (rx, ry, theta) — dùng chung cho xe chính + ghost.
struct ChassisQuad {
  Vector2 front_left;
  Vector2 front_right;
  Vector2 rear_right;
  Vector2 rear_left;
};

ChassisQuad locate_chassis_corners(float rx, float ry, float theta) {
  const float cos_t = std::cos(theta);
  const float sin_t = std::sin(theta);
  const float hl = kLayout.half_length_mm;
  const float hw = kLayout.half_width_mm;
  return {
    {rx + hl * cos_t + hw * sin_t, ry + hl * sin_t - hw * cos_t},
    {rx + hl * cos_t - hw * sin_t, ry + hl * sin_t + hw * cos_t},
    {rx - hl * cos_t - hw * sin_t, ry - hl * sin_t + hw * cos_t},
    {rx - hl * cos_t + hw * sin_t, ry - hl * sin_t - hw * cos_t},
  };
}

constexpr int kWindowWidth = 1920;
constexpr int kWindowHeight = 1080;
constexpr float kTopDockHeight = 3.0F;
constexpr float kBottomDockHeight = 10.2F;

// Bảng màu Neutral công nghiệp (Hallmark & Design-Taste)
constexpr Color kBgCharcoal = {14, 16, 19, 255};      // #0e1013
constexpr Color kBorderGray = {39, 39, 42, 255};      // #27272a
constexpr Color kTextMuted = {161, 161, 170, 255};    // #a1a1aa
constexpr Color kTextPrimary = {244, 244, 245, 255};  // #f4f4f5
constexpr Color kZincWhite = {250, 250, 250, 255};
constexpr Color kAmberAlert = {245, 158, 11, 255};  // #f59e0b

// Ticket 31: hệ font CSS-rem-like, neo 1080p = 18px, clamp [14, 38] tránh cực đoan.
inline constexpr float kRemBasePx = 18.0f;
inline constexpr float kRemRefHeight = 1080.0f;

float scale_font(float screen_h) {
  return std::clamp(screen_h * (kRemBasePx / kRemRefHeight), 14.0f, 38.0f);
}

// Shared speed scale for the trail heatmap: one scale for both runs, never
// normalised per run, so equal speeds always get equal colours.
constexpr float kSpeedScaleMinMmS = 0.0F;
constexpr float kSpeedScaleMaxMmS = 500.0F;

Color speed_color(float speed_mm_s) {
  const float clamped = std::clamp(speed_mm_s, kSpeedScaleMinMmS, kSpeedScaleMaxMmS);
  const float t = (clamped - kSpeedScaleMinMmS) / (kSpeedScaleMaxMmS - kSpeedScaleMinMmS);
  const auto channel = [t](float low, float high) {
    return static_cast<unsigned char>(low + (high - low) * t);
  };
  return {channel(56.0F, 250.0F), channel(120.0F, 250.0F), channel(190.0F, 220.0F), 235};
}

const char* describe_race_phase(const TrialFrame& frame) {
  if (frame.status.reason == StopReason::kNotReady) {
    return "NOT READY";
  }
  if (frame.status.phase == RacePhase::kFault) {
    return "FAULT";
  }
  if (frame.status.phase == RacePhase::kRunning) {
    return frame.run == 2 ? "RUN 2" : "RUN 1";
  }
  if (frame.status.phase == RacePhase::kDone) {
    return "STOPPED";
  }
  return "IDLE";
}

void draw_range_reading(const TrialFrame& frame, int x, int y, float rem) {
  const auto& tof = frame.measured_tof;
  const bool valid = tof.status == ReadingStatus::kValid && std::isfinite(tof.value);
  const bool no_return = tof.status == ReadingStatus::kNoReturn && std::isfinite(tof.value);
  const char* label = "No sample";
  if (valid) {
    label = TextFormat("%.0f mm", tof.value);
  } else if (no_return) {
    label = "No return";
  }
  DrawText(label, x, y, static_cast<int>(1.4F * rem), kTextPrimary);
  const double age_ms = frame.outcome_time_us > tof.time_us
                          ? static_cast<double>(frame.outcome_time_us - tof.time_us) * 1e-3
                          : 0.0;
  DrawText(
    valid || no_return ? TextFormat("Sample age %.0f ms", age_ms) : "Waiting for acquisition", x,
    y + static_cast<int>(2.0F * rem), static_cast<int>(rem), kTextMuted);
  DrawText(TextFormat("Fixture max %.1f m / %.0f deg", kLayout.tof_range_mm * 0.001F,
                      2.0F * kLayout.tof_half_fov_rad * RAD2DEG),
           x, y + static_cast<int>(3.5F * rem), static_cast<int>(rem), kTextMuted);
}

}  // namespace

bool Renderer::init(const TrackAsset& track) {
  SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT);
  InitWindow(kWindowWidth, kWindowHeight, "LF-Bot 2026 Autonomous Racing Platform");
  if (!IsWindowReady()) {
    return false;
  }
  SetWindowMinSize(1280, 720);
  SetTargetFPS(60);
  if (reload_track(track)) {
    return true;
  }
  shutdown();
  return false;
}

bool Renderer::reload_track(const TrackAsset& track) {
  if (track.grayscale.empty()) {
    return false;
  }
  // Raylib accepts mutable image storage; keep the CPU scene immutable.
  std::vector<std::uint8_t> pixels = track.grayscale;
  const Image image{
    pixels.data(), track.image_width, track.image_height, 1, PIXELFORMAT_UNCOMPRESSED_GRAYSCALE,
  };
  const Texture2D candidate = LoadTextureFromImage(image);
  if (candidate.id == 0) {
    return false;
  }
  constexpr float kMaxTrailPixels = 4096.0F;
  const float scale = std::min(
    {1.0F, kMaxTrailPixels / track.arena_width_mm, kMaxTrailPixels / track.arena_height_mm});
  const RenderTexture2D trail =
    LoadRenderTexture(std::max(1, static_cast<int>(track.arena_width_mm * scale)),
                      std::max(1, static_cast<int>(track.arena_height_mm * scale)));
  if (trail.id == 0) {
    UnloadTexture(candidate);
    return false;
  }
  if (texture_.id != 0) {
    UnloadTexture(texture_);
  }
  if (trail_texture_.id != 0) {
    UnloadRenderTexture(trail_texture_);
  }
  texture_ = candidate;
  trail_texture_ = trail;
  trail_camera_ = {{0.0F, 0.0F}, {0.0F, 0.0F}, 0.0F, scale};
  baked_frames_ = 0;
  baked_generation_ = 0;
  baked_run_ = 0;
  BeginTextureMode(trail_texture_);
  ClearBackground(BLANK);
  EndTextureMode();
  TrialFrame initial;
  initial.pose.x_mm = track.start_x_mm;
  initial.pose.y_mm = track.start_y_mm;
  initial.pose.yaw_rad = track.start_yaw_rad;
  update_camera(track, initial, 0.0F, true);
  return true;
}

void Renderer::shutdown() {
  if (texture_.id != 0) {
    UnloadTexture(texture_);
  }
  if (trail_texture_.id != 0) {
    UnloadRenderTexture(trail_texture_);
  }
  texture_ = {};
  trail_texture_ = {};
  if (IsWindowReady()) {
    CloseWindow();
  }
}

bool Renderer::should_close() {
  return WindowShouldClose();
}

void Renderer::toggle_camera_mode() {
  chase_camera_ = !chase_camera_;
}

void Renderer::update_camera(const TrackAsset& track, const TrialFrame& frame, float elapsed_s,
                             bool snap) {
  const auto screen_w = static_cast<float>(GetScreenWidth());
  const auto screen_h = static_cast<float>(GetScreenHeight());
  const float rem = scale_font(screen_h);
  const float dock_total_h = (kTopDockHeight + kBottomDockHeight + 1.0F) * rem;
  overview_scale_ = std::min((screen_w - 80.0F) / track.arena_width_mm,
                             (screen_h - dock_total_h) / track.arena_height_mm);
  if (chase_camera_) {
    const float lookahead = std::max(frame.pose.speed_mm_s * 0.12F, 60.0F);
    const Vector2 goal{
      frame.pose.x_mm + std::cos(frame.pose.yaw_rad) * lookahead,
      frame.pose.y_mm + std::sin(frame.pose.yaw_rad) * lookahead,
    };
    const float alpha = std::clamp(elapsed_s * 7.0F, 0.0F, 1.0F);
    if (snap) {
      smooth_target_ = goal;
    } else {
      smooth_target_.x += (goal.x - smooth_target_.x) * alpha;
      smooth_target_.y += (goal.y - smooth_target_.y) * alpha;
    }
    camera_.target = smooth_target_;
    camera_.zoom = 1.6F;
  } else {
    camera_.target = {track.arena_width_mm * 0.5F, track.arena_height_mm * 0.5F};
    camera_.zoom = overview_scale_;
  }
  camera_.offset = {
    screen_w * 0.5F,
    (screen_h + (kTopDockHeight - kBottomDockHeight) * rem) * 0.5F,
  };
  camera_.rotation = 0.0F;
}

void Renderer::draw_ghost_chassis(float x_mm, float y_mm, float yaw_rad) {
  const float hl = kLayout.half_length_mm;
  const float hw = kLayout.half_width_mm;
  const ChassisQuad q = locate_chassis_corners(x_mm, y_mm, yaw_rad);

  DrawRectanglePro(Rectangle{x_mm, y_mm, hl * 2.0f, hw * 2.0f}, Vector2{hl, hw},
                   yaw_rad * 180.0f / kPi, Color{39, 39, 42, 60});
  DrawLineEx(q.front_left, q.front_right, 1.8f, Color{113, 113, 122, 100});
  DrawLineEx(q.front_right, q.rear_right, 1.8f, Color{113, 113, 122, 100});
  DrawLineEx(q.rear_right, q.rear_left, 1.8f, Color{113, 113, 122, 100});
  DrawLineEx(q.rear_left, q.front_left, 1.8f, Color{113, 113, 122, 100});
}

void Renderer::draw_tof(const TrialFrame& frame) const {
  const auto& tof = frame.measured_tof;
  if ((tof.status != ReadingStatus::kValid && tof.status != ReadingStatus::kNoReturn) ||
      !std::isfinite(tof.value)) {
    return;
  }
  const float theta = frame.pose.yaw_rad;
  const Vector2 forward{std::cos(theta), std::sin(theta)};
  const Vector2 front{
    frame.pose.x_mm + kLayout.tof_forward_mm * forward.x,
    frame.pose.y_mm + kLayout.tof_forward_mm * forward.y,
  };
  const float radius = std::clamp(tof.value, 0.0F, kLayout.tof_range_mm);
  const float heading = theta * RAD2DEG;
  const float half_fov = kLayout.tof_half_fov_rad * RAD2DEG;
  DrawCircleSector(front, radius, heading - half_fov, heading + half_fov, 32,
                   Color{56, 189, 248, 10});
  DrawCircleSectorLines(front, radius, heading - half_fov, heading + half_fov, 32,
                        Color{56, 189, 248, 100});
  const Vector2 end{front.x + radius * forward.x, front.y + radius * forward.y};
  DrawLineEx(front, end, 1.0F / camera_.zoom, Color{56, 189, 248, 140});
  if (tof.status == ReadingStatus::kValid) {
    DrawCircleV(end, 3.0F / camera_.zoom, Color{56, 189, 248, 255});
  }
  // Metre ticks and the endpoint share the world scale, not the screen size.
  for (int metre = 1; static_cast<float>(metre) * 1000.0F < radius; ++metre) {
    const float distance = static_cast<float>(metre) * 1000.0F;
    const Vector2 tick{front.x + distance * forward.x, front.y + distance * forward.y};
    const float size = 5.0F / camera_.zoom;
    DrawLineEx({tick.x - forward.y * size, tick.y + forward.x * size},
               {tick.x + forward.y * size, tick.y - forward.x * size}, 1.0F / camera_.zoom,
               Color{56, 189, 248, 180});
  }
}

void Renderer::draw_chassis(const TrialFrame& frame) {
  const float rx = frame.pose.x_mm;
  const float ry = frame.pose.y_mm;
  const float theta = frame.pose.yaw_rad;
  const float cos_t = std::cos(theta);
  const float sin_t = std::sin(theta);
  const Vector2 front{rx + kLayout.tof_forward_mm * cos_t, ry + kLayout.tof_forward_mm * sin_t};
  DrawCircleV(front, 3.2F, Color{56, 189, 248, 255});
  const ChassisQuad quad = locate_chassis_corners(rx, ry, theta);
  DrawRectanglePro({rx, ry, kLayout.half_length_mm * 2.0F, kLayout.half_width_mm * 2.0F},
                   {kLayout.half_length_mm, kLayout.half_width_mm}, theta * RAD2DEG,
                   Color{24, 24, 27, 240});
  DrawLineEx(quad.front_left, quad.front_right, 1.8F, Color{244, 244, 245, 230});
  DrawLineEx(quad.front_right, quad.rear_right, 1.8F, Color{244, 244, 245, 230});
  DrawLineEx(quad.rear_right, quad.rear_left, 1.8F, Color{244, 244, 245, 230});
  DrawLineEx(quad.rear_left, quad.front_left, 1.8F, Color{244, 244, 245, 230});
  for (std::size_t i = 0; i < kLayout.ir_positions_mm.size(); ++i) {
    const auto& position = kLayout.ir_positions_mm[i];
    const Vector2 sensor{
      rx + position.x * cos_t + position.y * sin_t,
      ry + position.x * sin_t - position.y * cos_t,
    };
    const auto& sample = frame.ir[i];
    const bool valid = sample.status == ReadingStatus::kValid;
    const float blackness = valid ? std::clamp(sample.value, 0.0F, 1.0F) : 0.0F;
    const auto light = static_cast<unsigned char>(63.0F + blackness * 187.0F);
    DrawCircleV(sensor, 3.5F, valid ? Color{light, light, light, 255} : kAmberAlert);
    if (!valid) {
      DrawLineEx({sensor.x - 3.0F, sensor.y - 3.0F}, {sensor.x + 3.0F, sensor.y + 3.0F}, 1.0F,
                 kBgCharcoal);
    }
  }
}

void Renderer::bake_trails(const std::vector<TrialFrame>& trace, const TrialFrame& frame,
                           const Playback& playback) {
  if (trail_texture_.id == 0) {
    return;
  }
  const std::size_t end =
    playback.seeking ? std::min(trace.size(), playback.frame_index + 1U) : trace.size();
  const bool reset = end < baked_frames_ || baked_generation_ != frame.track_generation ||
                     baked_run_ != playback.selected_run;
  BeginTextureMode(trail_texture_);
  if (reset) {
    ClearBackground(BLANK);
    baked_frames_ = 0;
  }
  BeginMode2D(trail_camera_);
  for (std::size_t i = std::max<std::size_t>(1, baked_frames_); i < end; ++i) {
    const auto& previous = trace[i - 1];
    const auto& next = trace[i];
    if (next.run != playback.selected_run || next.run != previous.run ||
        next.track_generation != previous.track_generation || next.kind != FrameKind::kDecision) {
      continue;
    }
    DrawLineEx({previous.pose.x_mm, previous.pose.y_mm}, {next.pose.x_mm, next.pose.y_mm}, 2.5F,
               speed_color(previous.pose.speed_mm_s));
  }
  EndMode2D();
  EndTextureMode();
  baked_frames_ = end;
  baked_generation_ = frame.track_generation;
  baked_run_ = playback.selected_run;
}

void Renderer::draw_world(const TrackAsset& track, const TrialFrame& frame,
                          const std::vector<TrialFrame>& trace) {
  DrawTexturePro(
    texture_, {0.0F, 0.0F, static_cast<float>(texture_.width), static_cast<float>(texture_.height)},
    {0.0F, 0.0F, track.arena_width_mm, track.arena_height_mm}, {0.0F, 0.0F}, 0.0F, WHITE);
  DrawRectangleLinesEx({0.0F, 0.0F, track.arena_width_mm, track.arena_height_mm}, 2.5F,
                       kBorderGray);
  const Rectangle finish{
    track.finish_min_x_mm,
    track.finish_min_y_mm,
    track.finish_max_x_mm - track.finish_min_x_mm,
    track.finish_max_y_mm - track.finish_min_y_mm,
  };
  DrawRectangleRec(finish, Color{244, 244, 245, 25});
  DrawRectangleLinesEx(finish, 2.0F, Color{244, 244, 245, 180});
  for (const auto& obstacle : track.obstacles) {
    const float cosine = std::cos(obstacle.angle_rad);
    const float sine = std::sin(obstacle.angle_rad);
    const std::array<Vector2, 4> corners = {
      {
        {-obstacle.width_mm * 0.5F, -obstacle.height_mm * 0.5F},
        {obstacle.width_mm * 0.5F, -obstacle.height_mm * 0.5F},
        {obstacle.width_mm * 0.5F, obstacle.height_mm * 0.5F},
        {-obstacle.width_mm * 0.5F, obstacle.height_mm * 0.5F},
      },
    };
    DrawRectanglePro({obstacle.x_mm, obstacle.y_mm, obstacle.width_mm, obstacle.height_mm},
                     {obstacle.width_mm * 0.5F, obstacle.height_mm * 0.5F},
                     obstacle.angle_rad * RAD2DEG, Color{39, 39, 42, 220});
    for (std::size_t i = 0; i < corners.size(); ++i) {
      const auto& first = corners[i];
      const auto& last = corners[(i + 1U) % corners.size()];
      DrawLineEx(
        {
          obstacle.x_mm + first.x * cosine - first.y * sine,
          obstacle.y_mm + first.x * sine + first.y * cosine,
        },
        {
          obstacle.x_mm + last.x * cosine - last.y * sine,
          obstacle.y_mm + last.x * sine + last.y * cosine,
        },
        2.0F, kAmberAlert);
    }
  }
  if (trail_texture_.id != 0) {
    DrawTexturePro(trail_texture_.texture,
                   {
                     0.0F,
                     0.0F,
                     static_cast<float>(trail_texture_.texture.width),
                     -static_cast<float>(trail_texture_.texture.height),
                   },
                   {0.0F, 0.0F, track.arena_width_mm, track.arena_height_mm}, {0.0F, 0.0F}, 0.0F,
                   WHITE);
  }
  if (frame.run == 2) {
    const auto ghost = std::find_if(trace.begin(), trace.end(), [&frame](const TrialFrame& sample) {
      return sample.run == 1 && sample.track_generation == frame.track_generation &&
             sample.time_us >= frame.time_us;
    });
    if (ghost != trace.end()) {
      draw_ghost_chassis(ghost->pose.x_mm, ghost->pose.y_mm, ghost->pose.yaw_rad);
    }
  }
  draw_tof(frame);
  draw_chassis(frame);
}

void Renderer::draw_top_dock(const TrackAsset& track, const TrialFrame& frame,
                             const Playback& playback) const {
  const auto width = static_cast<float>(GetScreenWidth());
  const float rem = scale_font(static_cast<float>(GetScreenHeight()));
  const int height = static_cast<int>(kTopDockHeight * rem);
  const int font = static_cast<int>(rem);
  const int y = (height - font) / 2;
  const int pad = static_cast<int>(1.3F * rem);
  DrawRectangle(0, 0, GetScreenWidth(), height, Color{14, 16, 19, 245});
  DrawLine(0, height, GetScreenWidth(), height, kBorderGray);
  DrawText(TextFormat("TRACK: %s", track.name.c_str()), pad, y, font, kZincWhite);
  DrawText(TextFormat("MODE: %s", describe_race_phase(frame)), static_cast<int>(width * 0.30F), y,
           font, kAmberAlert);
  DrawText(TextFormat("t: %.2fs | v: %.2fm/s", static_cast<double>(frame.time_us) * 1e-6,
                      static_cast<double>(frame.pose.speed_mm_s) * 0.001),
           static_cast<int>(width * 0.49F), y, font, kTextPrimary);
  const char* camera = chase_camera_ ? "CHASE CAM" : "OVERVIEW";
  const char* status = playback.paused ? "PAUSED" : "RUNNING";
  const char* label = TextFormat("%s | %.2fx | %s", camera, playback.speed, status);
  DrawText(label, GetScreenWidth() - MeasureText(label, font) - pad, y, font,
           playback.paused ? kAmberAlert : kTextMuted);
}

void Renderer::draw_telemetry_dock(const TrialFrame& frame, const Playback& playback,
                                   std::size_t trace_size) {
  const int width = GetScreenWidth();
  const float rem = scale_font(static_cast<float>(GetScreenHeight()));
  const int height = static_cast<int>(kBottomDockHeight * rem);
  const int top = GetScreenHeight() - height;
  const int font = static_cast<int>(rem);
  const int pad = static_cast<int>(1.3F * rem);
  const int heading_y = top + static_cast<int>(0.8F * rem);
  const int row_y = top + static_cast<int>(2.8F * rem);
  const int motor_x = static_cast<int>(0.16F * static_cast<float>(width));
  const int ir_x = static_cast<int>(0.35F * static_cast<float>(width));
  const int tof_x = static_cast<int>(0.72F * static_cast<float>(width));
  DrawRectangle(0, top, width, height, kBgCharcoal);
  DrawLine(0, top, width, top, kBorderGray);
  DrawText("ROBOT / FRONT", pad, heading_y, font, kTextMuted);
  DrawText("MOTOR DUTY", motor_x, heading_y, font, kTextMuted);
  DrawText("IR / BLACKNESS", ir_x, heading_y, font, kTextMuted);
  DrawText("FRONT ToF", tof_x, heading_y, font, kTextMuted);

  TrialFrame preview = frame;
  preview.pose = {0.0F, 0.0F, -kPi * 0.5F, 0.0F};
  const Camera2D preview_camera{
    {static_cast<float>(pad) + 3.8F * rem, static_cast<float>(top) + 5.0F * rem},
    {0.0F, 0.0F},
    0.0F,
    5.5F * rem / (2.0F * kLayout.half_length_mm),
  };
  BeginMode2D(preview_camera);
  draw_chassis(preview);
  EndMode2D();

  for (int side = 0; side < 2; ++side) {
    const float duty = side == 0 ? frame.output.left_duty : frame.output.right_duty;
    const int y = row_y + static_cast<int>(static_cast<float>(side) * 2.4F * rem);
    DrawText(TextFormat("%s  %+4.0f%%", side == 0 ? "L" : "R", duty * 100.0F), motor_x, y, font,
             kTextPrimary);
    const int bar_y = y + static_cast<int>(1.3F * rem);
    const int bar_width = static_cast<int>(10.0F * rem);
    const int bar_height = static_cast<int>(0.4F * rem);
    const int middle = motor_x + bar_width / 2;
    DrawRectangle(motor_x, bar_y, bar_width, bar_height, kBorderGray);
    const float bounded = std::isfinite(duty) ? std::clamp(duty, -1.0F, 1.0F) : 0.0F;
    const int length = static_cast<int>(std::abs(bounded) * static_cast<float>(bar_width) * 0.5F);
    DrawRectangle(bounded < 0.0F ? middle - length : middle, bar_y, length, bar_height,
                  bounded < 0.0F ? kAmberAlert : kZincWhite);
    DrawLine(middle, bar_y - 2, middle, bar_y + bar_height + 2, kTextMuted);
  }

  const int channel_step = static_cast<int>(2.4F * rem);
  for (std::size_t i = 0; i < frame.ir.size(); ++i) {
    const auto& sample = frame.ir[i];
    const bool valid = sample.status == ReadingStatus::kValid && std::isfinite(sample.value);
    const int x = ir_x + static_cast<int>(i) * channel_step;
    DrawText(TextFormat("%zu", i), x, row_y, font, kTextMuted);
    const int bar_y = row_y + static_cast<int>(1.5F * rem);
    const int bar_width = static_cast<int>(1.8F * rem);
    const int bar_height = static_cast<int>(0.6F * rem);
    DrawRectangle(x, bar_y, bar_width, bar_height, kBorderGray);
    if (valid) {
      DrawRectangle(
        x, bar_y,
        static_cast<int>(std::clamp(sample.value, 0.0F, 1.0F) * static_cast<float>(bar_width)),
        bar_height, kZincWhite);
    }
    DrawText(valid ? TextFormat("%.1f", sample.value) : "--", x,
             row_y + static_cast<int>(2.8F * rem), font, valid ? kTextPrimary : kAmberAlert);
  }

  draw_range_reading(frame, tof_x, row_y, rem);

  const int footer_y = top + static_cast<int>(8.5F * rem);
  DrawLine(pad, footer_y - static_cast<int>(0.7F * rem), width - pad,
           footer_y - static_cast<int>(0.7F * rem), kBorderGray);
  const char* trace_label =
    TextFormat("TRACE %u  %zu/%zu", playback.selected_run,
               trace_size == 0 ? 0U : playback.frame_index + 1U, trace_size);
  DrawText(trace_label, ir_x, row_y + static_cast<int>(4.2F * rem), font,
           playback.seeking ? kAmberAlert : kTextMuted);
  DrawText(
    "SPACE pause   ENTER confirm/next run   R reset   TAB track   C camera   F full   1/2 trace   "
    "arrows seek   SHIFT x10   ,/. speed",
    pad, footer_y, font, kTextMuted);
}

void Renderer::draw(const TrackAsset& track, const TrialFrame& frame,
                    const std::vector<TrialFrame>& trace, const Playback& playback) {
  bake_trails(trace, frame, playback);
  BeginDrawing();
  ClearBackground(kBgCharcoal);
  BeginMode2D(camera_);
  draw_world(track, frame, trace);
  EndMode2D();
  const float rem = scale_font(static_cast<float>(GetScreenHeight()));
  const int font = static_cast<int>(rem);
  const int pad = static_cast<int>(1.3F * rem);
  const int scale_y = GetScreenHeight() - static_cast<int>((kBottomDockHeight + 1.5F) * rem);
  const float scale_mm = chase_camera_ ? 100.0F : 1000.0F;
  const int scale_width = static_cast<int>(scale_mm * camera_.zoom);
  DrawRectangle(pad - 6, scale_y - font - 6, scale_width + 12, font + 18, kBgCharcoal);
  DrawText(TextFormat("%.0f mm", scale_mm), pad, scale_y - font - 2, font, kTextPrimary);
  DrawLine(pad, scale_y + 4, pad + scale_width, scale_y + 4, kTextPrimary);
  DrawLine(pad, scale_y, pad, scale_y + 8, kTextPrimary);
  DrawLine(pad + scale_width, scale_y, pad + scale_width, scale_y + 8, kTextPrimary);
  const int legend_x = GetScreenWidth() - static_cast<int>(17.0F * rem);
  DrawRectangle(legend_x - 6, scale_y - font - 6, static_cast<int>(16.0F * rem), font + 18,
                kBgCharcoal);
  DrawText(TextFormat("TRAIL 0-%.0f mm/s", kSpeedScaleMaxMmS), legend_x, scale_y - font - 2, font,
           kTextMuted);
  for (int i = 0; i < 5; ++i) {
    DrawRectangle(legend_x + i * static_cast<int>(3.0F * rem), scale_y,
                  static_cast<int>(3.0F * rem), 6,
                  speed_color(kSpeedScaleMaxMmS * static_cast<float>(i) / 4.0F));
  }
  draw_top_dock(track, frame, playback);
  draw_telemetry_dock(frame, playback, trace.size());
  EndDrawing();
}

}  // namespace lf::sim
