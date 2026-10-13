#pragma once

#include <raylib.h>

#include <array>
#include <cstddef>
#include <vector>

#include "drive/types.hpp"
#include "sim_config.hpp"

namespace lf::sim {

// Điểm lưu vết quỹ đạo phục vụ so sánh Run 1/2 và video playback scrubber.
struct TracePoint {
  float x = 0.0f;
  float y = 0.0f;
  float theta = 0.0f;
  float v = 0.0f;
  float line_offset = 0.0f;
  float line_certainty = 0.0f;
  float left_duty = 0.0f;
  float right_duty = 0.0f;
  float tof_front = 300.0f;
  float tof_left = 300.0f;
  float tof_right = 300.0f;
  float elapsed_s = 0.0f;
  float speed_limit = 1.0f;
  float obstacle_side = 0.0f;
  float line_curve_val = 0.0f;  // κ tại thời điểm này (ticket 32)
  float tick = 0.0f;
  std::array<float, 8> ir_weights{};
};

// Khung thông tin telemetry truyền vào renderer mỗi frame.
struct RenderFrame {
  // Trạng thái vật lý & cảm biến hiện tại
  float x = 0.0f;
  float y = 0.0f;
  float theta = 0.0f;
  float v = 0.0f;
  float line_offset = 0.0f;
  float line_certainty = 0.0f;
  float tof_front = 300.0f;
  float tof_left = 300.0f;
  float tof_right = 300.0f;
  std::array<float, 8> ir_weights{};
  float left_duty = 0.0f;
  float right_duty = 0.0f;
  bool brake = false;
  float curvature = 0.0f;    // κ hiện tại → Predictive Arc + nhãn R (ticket 32)
  float speed_limit = 1.0f;  // để tô màu arc (phanh vs đủ ga)

  // Nhiệm vụ & trọng tài
  int mode = 0;  // lf::drive::RunMode
  float elapsed_s = 0.0f;
  float lap1_s = 0.0f;
  float lap2_s = 0.0f;
  bool dq = false;
  const char* dq_reason = nullptr;

  // Điều khiển & Playback bàn phím
  bool paused = false;
  float playback_speed = 1.0f;
  int scrub_frame = -1;  // -1: live; >= 0: frame đang tua
  int total_frames = 0;
};

// View thuần: đọc const, không sở hữu state vật lý.
class Renderer {
public:
  void init(const TrackManifest& manifest);
  void reload_track(const TrackManifest& manifest);
  void shutdown();

  // Cập nhật mượt mà camera
  void update_camera(float rx, float ry, float theta, float v, float dt,
                     const TrackManifest& manifest, bool snap);

  // Vẽ toàn bộ thế giới và telemetry HUD.
  void draw(const TrackManifest& manifest, const RenderFrame& frame,
            const lf::drive::RacePlan& plan, const std::vector<TracePoint>& run1_trace,
            const std::vector<TracePoint>& run2_trace);

  // Điều khiển camera
  void toggle_camera_mode();
  [[nodiscard]] bool is_chase_camera() const { return chase_camera_; }

  [[nodiscard]] static bool should_close();
  static void begin_frame();
  static void end_frame();

private:
  void draw_world(const TrackManifest& manifest, const RenderFrame& frame,
                  const std::vector<TracePoint>& run1_trace);
  static void draw_chassis(float rx, float ry, float theta, float curvature, float speed_limit,
                           const std::array<float, 8>& ir_weights, float d_front, float d_left,
                           float d_right);
  static void draw_predictive_arc(float rx, float ry, float theta, float curvature,
                                  float speed_limit);
  static void draw_ghost_chassis(float rx, float ry, float theta);

  // Bake quỹ đạo vào texture (chỉ vẽ điểm MỚI mỗi frame, blit 1 lệnh).
  void bake_trails(const std::vector<TracePoint>& run1_trace,
                   const std::vector<TracePoint>& run2_trace, const lf::drive::RacePlan& plan);
  void bake_segments(const std::vector<TracePoint>& trace, std::size_t& count, bool heatmap,
                     bool is_run1);

  void draw_hud(const TrackManifest& manifest, const RenderFrame& frame) const;
  void draw_top_dock(const TrackManifest& manifest, const RenderFrame& frame) const;
  static void draw_telemetry_dock(const RenderFrame& frame);

  Texture2D texture_{};
  Camera2D camera_{};
  bool chase_camera_ = true;
  Vector2 smooth_target_{};
  float overview_scale_ = 1.0f;

  // Texture quỹ đạo (world-space, 1 px = 1/trail_scale_ mm) + trạng thái đã bake.
  RenderTexture2D trail_rt_{};
  Camera2D trail_cam_{};
  float trail_scale_ = 1.0f;
  std::size_t baked_run1_ = 0;
  std::size_t baked_run2_ = 0;
  bool baked_ready_ = false;
};

}  // namespace lf::sim
