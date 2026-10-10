#pragma once

#include <string>
#include <vector>

#include "sim_config.hpp"

namespace lf::sim {

// Sở hữu bitmap grayscale + manifest track (resource adapter, chỉ dùng trong sim).
class TrackAsset {
public:
  bool load(const std::string& manifest_path);

  [[nodiscard]] unsigned char sample_grayscale(float x_mm, float y_mm) const;
  [[nodiscard]] const TrackManifest& manifest() const { return manifest_; }
  [[nodiscard]] int width() const { return width_; }
  [[nodiscard]] int height() const { return height_; }

private:
  TrackManifest manifest_;
  std::vector<unsigned char> pixels_;
  int width_ = 0;
  int height_ = 0;
};

}
