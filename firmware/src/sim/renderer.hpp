#pragma once

#include <raylib.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#include "track_asset.hpp"
#include "trial_frame.hpp"

namespace lf::sim {

struct Playback {
  bool paused = true;
  bool seeking = false;
  float speed = 1.0F;
  std::uint8_t selected_run = 1;
  std::size_t frame_index = 0;
};

class Renderer {
public:
  Renderer() = default;
  Renderer(const Renderer&) = delete;
  Renderer& operator=(const Renderer&) = delete;
  Renderer(Renderer&&) = delete;
  Renderer& operator=(Renderer&&) = delete;
  ~Renderer() { shutdown(); }

  [[nodiscard]] bool init(const TrackAsset& track);
  [[nodiscard]] bool reload_track(const TrackAsset& track);
  void shutdown();
  void update_camera(const TrackAsset& track, const TrialFrame& frame, float elapsed_s, bool snap);
  void draw(const TrackAsset& track, const TrialFrame& frame, const std::vector<TrialFrame>& trace,
            const Playback& playback);
  void toggle_camera_mode();
  [[nodiscard]] static bool should_close();

private:
  void draw_world(const TrackAsset& track, const TrialFrame& frame,
                  const std::vector<TrialFrame>& trace);
  static void draw_chassis(const TrialFrame& frame);
  void draw_tof(const TrialFrame& frame) const;
  static void draw_ghost_chassis(float x_mm, float y_mm, float yaw_rad);
  void bake_trails(const std::vector<TrialFrame>& trace, const TrialFrame& frame,
                   const Playback& playback);
  void draw_top_dock(const TrackAsset& track, const TrialFrame& frame,
                     const Playback& playback) const;
  static void draw_telemetry_dock(const TrialFrame& frame, const Playback& playback,
                                  std::size_t trace_size);

  Texture2D texture_{};
  Camera2D camera_{};
  bool chase_camera_ = true;
  Vector2 smooth_target_{};
  float overview_scale_ = 1.0F;
  RenderTexture2D trail_texture_{};
  Camera2D trail_camera_{};
  std::size_t baked_frames_ = 0;
  std::uint32_t baked_generation_ = 0;
  std::uint8_t baked_run_ = 0;
};

}  // namespace lf::sim
