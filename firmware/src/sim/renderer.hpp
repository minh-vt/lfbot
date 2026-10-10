#pragma once

#include <raylib.h>

#include "sim_config.hpp"

namespace lf::sim {

// View thuần: đọc const, không giữ state vật lý.
class Renderer {
public:
  void init(const TrackManifest& manifest);
  void shutdown();
  void draw(const TrackManifest& manifest, float rx, float ry, float theta_rad, int mode,
            float elapsed_s, float speed_mps, float lap1_s, float lap2_s, bool dq);

  [[nodiscard]] static bool should_close();
  static void begin_frame();
  static void end_frame();

private:
  Texture2D texture_{};
  float scale_ = 1.0f;
};

}
