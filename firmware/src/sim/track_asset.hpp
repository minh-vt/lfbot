#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace lf::sim {

struct Obstacle {
  float x_mm = 0.0F;
  float y_mm = 0.0F;
  float width_mm = 0.0F;
  float height_mm = 0.0F;
  float angle_rad = 0.0F;
};

struct TrackAsset {
  std::string name;
  std::string image_path;
  std::vector<std::uint8_t> grayscale;
  int image_width = 0;
  int image_height = 0;
  float arena_width_mm = 0.0F;
  float arena_height_mm = 0.0F;
  float start_x_mm = 0.0F;
  float start_y_mm = 0.0F;
  float start_yaw_rad = 0.0F;
  float finish_min_x_mm = 0.0F;
  float finish_max_x_mm = 0.0F;
  float finish_min_y_mm = 0.0F;
  float finish_max_y_mm = 0.0F;
  std::vector<Obstacle> obstacles;

  [[nodiscard]] bool load(const std::string& manifest, std::string& error);
  [[nodiscard]] std::uint8_t sample(float x_mm, float y_mm) const noexcept;
};

}  // namespace lf::sim
