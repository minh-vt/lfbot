#include "renderer.hpp"

#include <raylib.h>

#include <algorithm>
#include <array>
#include <cstdio>

namespace lf::sim {
namespace {

constexpr int kWinW = 1920;
constexpr int kWinH = 1080;

const char* mode_name(int mode) {
  switch (mode) {
    case 0:
      return "Idle";
    case 1:
      return "Run 1 (Map)";
    case 2:
      return "Run 2 (Sprint)";
    default:
      return "Done";
  }
}

}

void Renderer::init(const TrackManifest& manifest) {
  const Image image = LoadImage(manifest.image_path.c_str());
  texture_ = LoadTextureFromImage(image);
  UnloadImage(image);
  scale_ = std::min(static_cast<float>(kWinW) / static_cast<float>(texture_.width),
                    static_cast<float>(kWinH) / static_cast<float>(texture_.height));
  InitWindow(kWinW, kWinH, "LF-Bot 2026 Sim");
  SetTargetFPS(60);
}

void Renderer::shutdown() {
  UnloadTexture(texture_);
  CloseWindow();
}

bool Renderer::should_close() {
  return WindowShouldClose();
}

void Renderer::begin_frame() {
  BeginDrawing();
}

void Renderer::draw(const TrackManifest& manifest, float rx, float ry, float theta_rad, int mode,
                    float elapsed_s, float speed_mps, float lap1_s, float lap2_s, bool dq) {
  ClearBackground(BLACK);
  DrawTextureEx(texture_, Vector2{0.0f, 0.0f}, 0.0f, scale_, WHITE);

  DrawRectangleLinesEx(
    Rectangle{
      manifest.finish_min_x * scale_,
      manifest.finish_min_y * scale_,
      (manifest.finish_max_x - manifest.finish_min_x) * scale_,
      (manifest.finish_max_y - manifest.finish_min_y) * scale_,
    },
    2.0f, GREEN);

  for (const auto& obstacle : manifest.obstacles) {
    DrawRectangleLinesEx(
      Rectangle{
        (obstacle.x_mm - obstacle.width_mm * 0.5f) * scale_,
        (obstacle.y_mm - obstacle.height_mm * 0.5f) * scale_,
        obstacle.width_mm * scale_,
        obstacle.height_mm * scale_,
      },
      2.0f, RED);
  }

  const float robot_w = kRobotHalfLengthMm * 2.0f * scale_;
  const float robot_h = kRobotHalfWidthMm * 2.0f * scale_;
  DrawRectanglePro(Rectangle{rx * scale_, ry * scale_, robot_w, robot_h},
                   Vector2{robot_w * 0.5f, robot_h * 0.5f}, theta_rad * 180.0f / kPi, BLUE);

  std::array<char, 256> text{};
  const int written =
    std::snprintf(text.data(), text.size(), "%s | t=%.1fs | v=%.2fm/s | Run1=%.2fs | Run2=%.2fs%s",
                  mode_name(mode), elapsed_s, speed_mps, lap1_s, lap2_s, dq ? " | DQ!" : "");
  if (written < 0) {
    text[0] = '\0';
  }
  DrawText(text.data(), 16, 16, 24, YELLOW);
  DrawText("Space=start  R=reset  1=Run1  2=Run2", 16, 48, 20, GRAY);
}

void Renderer::end_frame() {
  EndDrawing();
}

}
