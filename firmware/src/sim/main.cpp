#include <raylib.h>
#include <raymath.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>

#include "config/robot_config.hpp"
#include "hud.hpp"
#include "sim_engine.hpp"
#include "telemetry_receiver.hpp"
#include "theme.hpp"
#include "track_manifest.hpp"
#include "types.hpp"
#include "world.hpp"

namespace {

lf::sim::GuiAction poll_mode_hotkeys(lf::sim::SimRobotState& state, char* toast_msg,
                                     size_t toast_size, float& toast_timer) {
  if (IsKeyPressed(KEY_SPACE)) {
    return lf::sim::GuiAction::kTogglePause;
  }
  if (IsKeyPressed(KEY_R)) {
    return lf::sim::GuiAction::kResetPose;
  }
  if (IsKeyPressed(KEY_E)) {
    return lf::sim::GuiAction::kToggleSetupMode;
  }
  if (IsKeyPressed(KEY_ENTER)) {
    return lf::sim::GuiAction::kCommitC2;
  }
  if (IsKeyPressed(KEY_ONE)) {
    return lf::sim::GuiAction::kRun1Map;
  }
  if (IsKeyPressed(KEY_TWO)) {
    return lf::sim::GuiAction::kRun2Sprint;
  }
  if (IsKeyPressed(KEY_T)) {
    return lf::sim::GuiAction::kAutoTune;
  }
  if (IsKeyPressed(KEY_P)) {
    return lf::sim::GuiAction::kToggleTuningDrawer;
  }
  if (IsKeyPressed(KEY_O)) {
    return lf::sim::GuiAction::kTriggerLiveReopt;
  }

  if (IsKeyPressed(KEY_F4)) {
    state.is_telemetry_mode = !state.is_telemetry_mode;
    if (state.is_telemetry_mode) {
      state.is_paused = false;
      state.trail.clear();
      state.packets_received = 0;
      (void)snprintf(toast_msg, toast_size,
                     "GROUND STATION: Listening on UDP 9876. Awaiting telemetry...");
      toast_timer = 4.0f;
    } else {
      (void)snprintf(toast_msg, toast_size, "LOCAL SIM: Returned to local simulation mode.");
      toast_timer = 3.0f;
    }
  }

  return lf::sim::GuiAction::kNone;
}

void handle_camera_hotkeys(Rectangle view_rect, Camera2D& camera, float ui_scale,
                           lf::sim::SimEngine& engine, bool& follow_robot, bool& is_user_zoomed,
                           float track_w, float track_h) {
  if (IsKeyPressed(KEY_C)) {
    follow_robot = !follow_robot;
    if (follow_robot) {
      is_user_zoomed = false;
      engine.get_state_mut().is_auto_scale_cam = false;
    }
  }
  if (IsKeyPressed(KEY_F)) {
    follow_robot = false;
    is_user_zoomed = false;
    engine.get_state_mut().is_auto_scale_cam = false;
    const float margin = 40.0f * ui_scale;
    camera.zoom = std::min((view_rect.width - 2.0f * margin) / track_w,
                           (view_rect.height - 2.0f * margin) / track_h);
    camera.target = Vector2{track_w * 0.5f, track_h * 0.5f};
    camera.offset = Vector2{view_rect.width * 0.5f, view_rect.height * 0.5f};
  }
}

void handle_authoring_shortcuts(Vector2 mouse_world, lf::sim::SimEngine& engine,
                                lf::sim::TrackManifest& manifest, const std::string& manifest_path,
                                char* toast_msg, size_t toast_size, float& toast_timer) {
  // Ctrl + S: Luu cau hinh sa ban
  if ((IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL)) && IsKeyPressed(KEY_S)) {
    manifest.obstacles = engine.get_obstacles();
    if (lf::sim::save_track_manifest(manifest_path, manifest)) {
      (void)snprintf(toast_msg, toast_size, "TRACK MANIFEST SAVED: %s", manifest_path.c_str());
      toast_timer = 3.0f;
    }
    return;
  }

  // S + Click: Dat Start Pose
  if (IsKeyDown(KEY_S) && !IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_RIGHT_CONTROL) &&
      IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
    manifest.start_x = mouse_world.x;
    manifest.start_y = mouse_world.y;
    const float th_rad = manifest.start_theta_deg * (lf::constants::kPi / 180.0f);
    engine.reset_pose(lf::RobotMode::kIdle, true, manifest.start_x, manifest.start_y, th_rad);
    (void)snprintf(toast_msg, toast_size, "START POSE SET: (%.0f, %.0f mm)", mouse_world.x,
                   mouse_world.y);
    toast_timer = 2.5f;
    return;
  }

  // O: Them chuong ngai vat tai vi tri con tro chuot
  if (IsKeyPressed(KEY_O)) {
    auto obstacles = engine.get_obstacles();
    obstacles.push_back(lf::sim::Obstacle{mouse_world, 105.0f, 60.0f, 0.0f});
    engine.set_obstacles(obstacles);
    manifest.obstacles = obstacles;
    (void)snprintf(toast_msg, toast_size, "ADDED OBSTACLE #%zu at (%.0f, %.0f)", obstacles.size(),
                   mouse_world.x, mouse_world.y);
    toast_timer = 2.0f;
    engine.recompute_racing_trajectory(manifest.start_x, manifest.start_y);
    return;
  }

  // Delete / Backspace: Xoa chuong ngai vat dang duoc chon/hover
  auto& state = engine.get_state_mut();
  if ((IsKeyPressed(KEY_DELETE) || IsKeyPressed(KEY_BACKSPACE)) &&
      state.hovered_obstacle_idx >= 0 &&
      state.hovered_obstacle_idx < static_cast<int>(engine.get_obstacles().size())) {
    auto obstacles = engine.get_obstacles();
    obstacles.erase(obstacles.begin() + state.hovered_obstacle_idx);
    engine.set_obstacles(obstacles);
    manifest.obstacles = std::move(obstacles);
    state.hovered_obstacle_idx = -1;
    (void)snprintf(toast_msg, toast_size, "DELETED OBSTACLE");
    toast_timer = 2.0f;
    engine.recompute_racing_trajectory(manifest.start_x, manifest.start_y);
  }
}

void update_hovered_obstacle(const Vector2& mouse_world,
                             const std::vector<lf::sim::Obstacle>& obstacles,
                             lf::sim::SimRobotState& state) {
  state.hovered_obstacle_idx = -1;
  for (size_t i = 0; i < obstacles.size(); ++i) {
    const auto& obs = obstacles.at(i);
    const float half_w = obs.width_mm * 0.5f;
    const float half_h = obs.height_mm * 0.5f;
    const float r = std::sqrt(half_w * half_w + half_h * half_h);
    const float dx = mouse_world.x - obs.pos.x;
    const float dy = mouse_world.y - obs.pos.y;
    if (std::sqrt(dx * dx + dy * dy) <= r) {
      state.hovered_obstacle_idx = static_cast<int>(i);
      break;
    }
  }
}

bool handle_obstacle_rotation(lf::sim::SimEngine& engine, lf::sim::TrackManifest& manifest,
                              float wheel, char* toast_msg, size_t toast_size, float& toast_timer) {
  auto& obstacles = engine.get_obstacles_mut();
  const auto& state = engine.get_state();
  if (state.hovered_obstacle_idx < 0 ||
      state.hovered_obstacle_idx >= static_cast<int>(obstacles.size())) {
    return false;
  }
  auto& obs = obstacles.at(static_cast<size_t>(state.hovered_obstacle_idx));
  bool changed = false;
  if (wheel != 0.0f) {
    const float step = (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) ? 15.0f : 5.0f;
    obs.angle_deg = std::fmod(obs.angle_deg + wheel * step + 360.0f, 360.0f);
    changed = true;
  }
  if (IsKeyPressed(KEY_Q)) {
    obs.angle_deg = std::fmod(obs.angle_deg - 15.0f + 360.0f, 360.0f);
    changed = true;
  }
  if (IsKeyPressed(KEY_E) && !IsKeyDown(KEY_LEFT_CONTROL)) {
    obs.angle_deg = std::fmod(obs.angle_deg + 15.0f + 360.0f, 360.0f);
    changed = true;
  }
  if (changed) {
    manifest.obstacles = obstacles;
    (void)snprintf(toast_msg, toast_size, "ROTATE OBSTACLE #%d: Angle %.0f°",
                   state.hovered_obstacle_idx + 1, obs.angle_deg);
    toast_timer = 1.5f;
    engine.recompute_racing_trajectory(manifest.start_x, manifest.start_y);
  }
  return true;
}

void handle_mouse_interactions(Rectangle view_rect, Camera2D& camera, lf::sim::SimEngine& engine,
                               lf::sim::TrackManifest& manifest, const std::string& manifest_path,
                               int& dragged_obstacle_idx, Vector2& drag_offset, bool& follow_robot,
                               bool& is_user_zoomed, char* toast_msg, size_t toast_size,
                               float& toast_timer) {
  const Vector2 mouse_pos = GetMousePosition();
  if (!CheckCollisionPointRec(mouse_pos, view_rect)) {
    return;
  }

  auto& state = engine.get_state_mut();

  // Protect HUD regions from canvas clicks
  const float top_hud_limit = (state.is_setup_mode ? 92.0f : 56.0f);
  const float bottom_hud_limit = view_rect.height - 48.0f;
  if (mouse_pos.y < top_hud_limit || mouse_pos.y > bottom_hud_limit) {
    return;
  }

  const Vector2 mouse_world = GetScreenToWorld2D(mouse_pos, camera);
  auto& obstacles = engine.get_obstacles_mut();

  handle_authoring_shortcuts(mouse_world, engine, manifest, manifest_path, toast_msg, toast_size,
                             toast_timer);
  update_hovered_obstacle(mouse_world, obstacles, state);

  // Setup mode active tools
  if (state.is_setup_mode && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
    if (state.setup_tool == lf::sim::SetupTool::kSetStart) {
      manifest.start_x = mouse_world.x;
      manifest.start_y = mouse_world.y;
      const float th_rad = manifest.start_theta_deg * (lf::constants::kPi / 180.0f);
      engine.reset_pose(lf::RobotMode::kIdle, true, manifest.start_x, manifest.start_y, th_rad);
      state.setup_tool = lf::sim::SetupTool::kSelect;
      (void)snprintf(toast_msg, toast_size, "START POSE SET: (%.0f, %.0f mm)", mouse_world.x,
                     mouse_world.y);
      toast_timer = 2.5f;
      return;
    }
    if (state.setup_tool == lf::sim::SetupTool::kSetFinish) {
      manifest.finish_box.min_x_mm = mouse_world.x - 80.0f;
      manifest.finish_box.max_x_mm = mouse_world.x + 80.0f;
      manifest.finish_box.min_y_mm = mouse_world.y - 80.0f;
      manifest.finish_box.max_y_mm = mouse_world.y + 80.0f;
      state.setup_tool = lf::sim::SetupTool::kSelect;
      (void)snprintf(toast_msg, toast_size, "FINISH BOX SET: (%.0f, %.0f mm)", mouse_world.x,
                     mouse_world.y);
      toast_timer = 2.5f;
      return;
    }
    if (state.setup_tool == lf::sim::SetupTool::kAddObstacle) {
      obstacles.push_back(lf::sim::Obstacle{mouse_world, 105.0f, 60.0f, 0.0f});
      engine.set_obstacles(obstacles);
      manifest.obstacles = obstacles;
      state.setup_tool = lf::sim::SetupTool::kSelect;
      (void)snprintf(toast_msg, toast_size, "ADDED OBSTACLE #%zu at (%.0f, %.0f)", obstacles.size(),
                     mouse_world.x, mouse_world.y);
      toast_timer = 2.0f;
      engine.recompute_racing_trajectory(manifest.start_x, manifest.start_y);
      return;
    }
  }

  // Right click deletes hovered obstacle in setup mode
  if (state.is_setup_mode && IsMouseButtonPressed(MOUSE_BUTTON_RIGHT) &&
      state.hovered_obstacle_idx >= 0 &&
      state.hovered_obstacle_idx < static_cast<int>(obstacles.size())) {
    obstacles.erase(obstacles.begin() + state.hovered_obstacle_idx);
    engine.set_obstacles(obstacles);
    manifest.obstacles = obstacles;
    state.hovered_obstacle_idx = -1;
    (void)snprintf(toast_msg, toast_size, "DELETED OBSTACLE");
    toast_timer = 2.0f;
    engine.recompute_racing_trajectory(manifest.start_x, manifest.start_y);
    return;
  }

  if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !IsKeyDown(KEY_S) &&
      state.hovered_obstacle_idx >= 0) {
    dragged_obstacle_idx = state.hovered_obstacle_idx;
    drag_offset = Vector2{
      mouse_world.x - obstacles.at(static_cast<size_t>(dragged_obstacle_idx)).pos.x,
      mouse_world.y - obstacles.at(static_cast<size_t>(dragged_obstacle_idx)).pos.y,
    };
  }

  if (dragged_obstacle_idx >= 0) {
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
      obstacles.at(static_cast<size_t>(dragged_obstacle_idx)).pos = Vector2{
        mouse_world.x - drag_offset.x,
        mouse_world.y - drag_offset.y,
      };
    } else {
      manifest.obstacles = obstacles;
      dragged_obstacle_idx = -1;
      engine.recompute_racing_trajectory(manifest.start_x, manifest.start_y);
    }
  }

  const float wheel = GetMouseWheelMove();
  if (!handle_obstacle_rotation(engine, manifest, wheel, toast_msg, toast_size, toast_timer) &&
      wheel != 0.0f) {
    is_user_zoomed = true;
    camera.offset = mouse_pos;
    camera.target = mouse_world;
    camera.zoom = std::clamp(camera.zoom + wheel * 0.08f * camera.zoom, 0.20f, 4.50f);
  }

  if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) &&
      (!state.is_setup_mode || state.hovered_obstacle_idx < 0)) {
    is_user_zoomed = true;
    const Vector2 delta = GetMouseDelta();
    camera.target.x -= delta.x / camera.zoom;
    camera.target.y -= delta.y / camera.zoom;
    follow_robot = false;
  }
}

void update_camera_viewport(Camera2D& camera, Rectangle view_rect, float ui_scale,
                            bool follow_robot, bool is_user_zoomed,
                            const lf::sim::SimRobotState& state, float track_w, float track_h) {
  const float view_w = view_rect.width;
  const float view_h = view_rect.height;

  if (state.is_auto_scale_cam || (state.is_telemetry_mode && !is_user_zoomed && !follow_robot)) {
    if (state.trail.size() >= 2) {
      float min_x = state.x;
      float max_x = state.x;
      float min_y = state.y;
      float max_y = state.y;
      for (const auto& pt : state.trail) {
        min_x = std::min(min_x, pt.x);
        max_x = std::max(max_x, pt.x);
        min_y = std::min(min_y, pt.y);
        max_y = std::max(max_y, pt.y);
      }
      const float margin = 160.0f * ui_scale;
      const float w_span = std::max(400.0f, (max_x - min_x) + 2.0f * margin);
      const float h_span = std::max(300.0f, (max_y - min_y) + 2.0f * margin);
      const float target_zoom =
        std::clamp(std::min((view_w - 2.0f * margin) / w_span, (view_h - 2.0f * margin) / h_span),
                   0.15f, 2.50f);
      const Vector2 target_center = Vector2{(min_x + max_x) * 0.5f, (min_y + max_y) * 0.5f};
      camera.offset = Vector2{view_w * 0.5f, view_h * 0.5f};
      camera.target.x += (target_center.x - camera.target.x) * 0.08f;
      camera.target.y += (target_center.y - camera.target.y) * 0.08f;
      camera.zoom += (target_zoom - camera.zoom) * 0.08f;
    } else {
      camera.offset = Vector2{view_w * 0.5f, view_h * 0.5f};
      camera.target.x += (state.x - camera.target.x) * 0.08f;
      camera.target.y += (state.y - camera.target.y) * 0.08f;
      camera.zoom = 1.0f;
    }
  } else if (follow_robot) {
    camera.offset = Vector2{view_w * 0.5f, view_h * 0.5f};
    camera.target.x += (state.x - camera.target.x) * 0.12f;
    camera.target.y += (state.y - camera.target.y) * 0.12f;
    camera.zoom = 1.6f;
  } else if (!is_user_zoomed) {
    const float margin = 40.0f * ui_scale;
    const float fit_zoom =
      std::min((view_w - 2.0f * margin) / track_w, (view_h - 2.0f * margin) / track_h);
    camera.offset = Vector2{view_w * 0.5f, view_h * 0.5f};
    camera.target = Vector2{track_w * 0.5f, track_h * 0.5f};
    camera.zoom = fit_zoom;
  }
}

void switch_to_track_manifest(size_t next_idx, size_t& current_idx,
                              const std::vector<std::string>& manifest_paths,
                              lf::sim::TrackManifest& current_manifest, lf::sim::SimEngine& engine,
                              Texture2D& track_texture, Camera2D& camera, Rectangle view_rect,
                              float ui_scale, bool& follow_robot, bool& is_user_zoomed,
                              char* toast_msg, size_t toast_size, float& toast_timer) {
  if (next_idx >= manifest_paths.size()) {
    return;
  }
  current_idx = next_idx;
  if (!lf::sim::load_track_manifest(manifest_paths.at(next_idx), current_manifest)) {
    (void)snprintf(toast_msg, toast_size, "ERROR: Could not load %s",
                   manifest_paths.at(next_idx).c_str());
    toast_timer = 3.0f;
    return;
  }

  UnloadTexture(track_texture);
  track_texture = LoadTexture(current_manifest.image_path.c_str());

  engine.load_manifest(current_manifest);

  const float track_w = engine.get_track_width_mm();
  const float track_h = engine.get_track_height_mm();

  follow_robot = false;
  is_user_zoomed = false;
  engine.get_state_mut().is_auto_scale_cam = false;
  const float margin = 40.0f * ui_scale;
  camera.zoom = std::min((view_rect.width - 2.0f * margin) / track_w,
                         (view_rect.height - 2.0f * margin) / track_h);
  camera.target = Vector2{track_w * 0.5f, track_h * 0.5f};
  camera.offset = Vector2{view_rect.width * 0.5f, view_rect.height * 0.5f};

  (void)snprintf(toast_msg, toast_size, "TRACK: %s (%.0fx%.0f mm)", current_manifest.name.c_str(),
                 track_w, track_h);
  toast_timer = 3.5f;
}

bool handle_tuning_action(lf::sim::GuiAction gui_act, lf::sim::SimEngine& engine,
                          const lf::sim::TrackManifest& manifest, char* toast_buf,
                          size_t toast_buf_len, float& toast_timer) {
  auto& robot = engine.get_robot_mut();
  auto& state = engine.get_state_mut();

  switch (gui_act) {
    case lf::sim::GuiAction::kToggleTuningDrawer: {
      state.is_tuning_drawer_open = !state.is_tuning_drawer_open;
      return true;
    }
    case lf::sim::GuiAction::kTuneKpUp: {
      float kp = 0.0f;
      float ki = 0.0f;
      float kd = 0.0f;
      robot.get_line_pid_gains(kp, ki, kd);
      robot.set_line_pid_gains(kp + 0.005f, ki, kd);
      (void)snprintf(toast_buf, toast_buf_len, "TUNING: Kp = %.4f", kp + 0.005f);
      toast_timer = 2.0f;
      return true;
    }
    case lf::sim::GuiAction::kTuneKpDown: {
      float kp = 0.0f;
      float ki = 0.0f;
      float kd = 0.0f;
      robot.get_line_pid_gains(kp, ki, kd);
      robot.set_line_pid_gains(std::max(0.0f, kp - 0.005f), ki, kd);
      (void)snprintf(toast_buf, toast_buf_len, "TUNING: Kp = %.4f", std::max(0.0f, kp - 0.005f));
      toast_timer = 2.0f;
      return true;
    }
    case lf::sim::GuiAction::kTuneKdUp: {
      float kp = 0.0f;
      float ki = 0.0f;
      float kd = 0.0f;
      robot.get_line_pid_gains(kp, ki, kd);
      robot.set_line_pid_gains(kp, ki, kd + 0.001f);
      (void)snprintf(toast_buf, toast_buf_len, "TUNING: Kd = %.4f", kd + 0.001f);
      toast_timer = 2.0f;
      return true;
    }
    case lf::sim::GuiAction::kTuneKdDown: {
      float kp = 0.0f;
      float ki = 0.0f;
      float kd = 0.0f;
      robot.get_line_pid_gains(kp, ki, kd);
      robot.set_line_pid_gains(kp, ki, std::max(0.0f, kd - 0.001f));
      (void)snprintf(toast_buf, toast_buf_len, "TUNING: Kd = %.4f", std::max(0.0f, kd - 0.001f));
      toast_timer = 2.0f;
      return true;
    }
    case lf::sim::GuiAction::kTuneVmapUp: {
      const float v_sprint = robot.get_config().profiler.max_straight_speed_mps;
      const float v_map = robot.get_config().profiler.safe_map_speed_mps;
      robot.set_tactical_speeds(std::min(1.50f, v_map + 0.05f), v_sprint);
      (void)snprintf(toast_buf, toast_buf_len, "TUNING: Map V = %.2f m/s",
                     std::min(1.50f, v_map + 0.05f));
      toast_timer = 2.0f;
      return true;
    }
    case lf::sim::GuiAction::kTuneVmapDown: {
      const float v_sprint = robot.get_config().profiler.max_straight_speed_mps;
      const float v_map = robot.get_config().profiler.safe_map_speed_mps;
      robot.set_tactical_speeds(std::max(0.20f, v_map - 0.05f), v_sprint);
      (void)snprintf(toast_buf, toast_buf_len, "TUNING: Map V = %.2f m/s",
                     std::max(0.20f, v_map - 0.05f));
      toast_timer = 2.0f;
      return true;
    }
    case lf::sim::GuiAction::kTuneVsprintUp: {
      const float v_sprint = robot.get_config().profiler.max_straight_speed_mps;
      const float v_map = robot.get_config().profiler.safe_map_speed_mps;
      robot.set_tactical_speeds(v_map, std::min(4.00f, v_sprint + 0.10f));
      (void)snprintf(toast_buf, toast_buf_len, "TUNING: Sprint V = %.2f m/s",
                     std::min(4.00f, v_sprint + 0.10f));
      toast_timer = 2.0f;
      return true;
    }
    case lf::sim::GuiAction::kTuneVsprintDown: {
      const float v_sprint = robot.get_config().profiler.max_straight_speed_mps;
      const float v_map = robot.get_config().profiler.safe_map_speed_mps;
      robot.set_tactical_speeds(v_map, std::max(0.50f, v_sprint - 0.10f));
      (void)snprintf(toast_buf, toast_buf_len, "TUNING: Sprint V = %.2f m/s",
                     std::max(0.50f, v_sprint - 0.10f));
      toast_timer = 2.0f;
      return true;
    }
    case lf::sim::GuiAction::kTuneAccelUp: {
      const float accel = robot.get_config().profiler.max_accel_mps2;
      robot.set_profiler_dynamics(accel + 0.50f, 4.5f, 0.8f);
      (void)snprintf(toast_buf, toast_buf_len, "TUNING: Accel = %.1f m/s²", accel + 0.50f);
      toast_timer = 2.0f;
      return true;
    }
    case lf::sim::GuiAction::kTuneAccelDown: {
      const float accel = robot.get_config().profiler.max_accel_mps2;
      robot.set_profiler_dynamics(std::max(1.0f, accel - 0.50f), 4.5f, 0.8f);
      (void)snprintf(toast_buf, toast_buf_len, "TUNING: Accel = %.1f m/s²",
                     std::max(1.0f, accel - 0.50f));
      toast_timer = 2.0f;
      return true;
    }
    case lf::sim::GuiAction::kTuneObsClearUp: {
      float obs_dist = 0.0f;
      float obs_clear = 0.0f;
      float obs_len = 0.0f;
      robot.get_obstacle_parameters(obs_dist, obs_clear, obs_len);
      robot.set_obstacle_parameters(obs_dist, obs_clear + 0.01f, obs_len);
      (void)snprintf(toast_buf, toast_buf_len, "TUNING: Y_clear = %.2f m", obs_clear + 0.01f);
      toast_timer = 2.0f;
      return true;
    }
    case lf::sim::GuiAction::kTuneObsClearDown: {
      float obs_dist = 0.0f;
      float obs_clear = 0.0f;
      float obs_len = 0.0f;
      robot.get_obstacle_parameters(obs_dist, obs_clear, obs_len);
      robot.set_obstacle_parameters(obs_dist, std::max(0.05f, obs_clear - 0.01f), obs_len);
      (void)snprintf(toast_buf, toast_buf_len, "TUNING: Y_clear = %.2f m",
                     std::max(0.05f, obs_clear - 0.01f));
      toast_timer = 2.0f;
      return true;
    }
    case lf::sim::GuiAction::kTuneJerkUp: {
      float max_dev = 0.0f;
      float jerk = 0.0f;
      robot.get_optimizer_params(max_dev, jerk);
      robot.set_optimizer_params(max_dev, std::min(0.50f, jerk + 0.01f));
      (void)snprintf(toast_buf, toast_buf_len, "TUNING: Jerk Damp w = %.3f",
                     std::min(0.50f, jerk + 0.01f));
      toast_timer = 2.0f;
      return true;
    }
    case lf::sim::GuiAction::kTuneJerkDown: {
      float max_dev = 0.0f;
      float jerk = 0.0f;
      robot.get_optimizer_params(max_dev, jerk);
      robot.set_optimizer_params(max_dev, std::max(0.005f, jerk - 0.01f));
      (void)snprintf(toast_buf, toast_buf_len, "TUNING: Jerk Damp w = %.3f",
                     std::max(0.005f, jerk - 0.01f));
      toast_timer = 2.0f;
      return true;
    }
    case lf::sim::GuiAction::kTuneCorridorUp: {
      float max_dev = 0.0f;
      float jerk = 0.0f;
      robot.get_optimizer_params(max_dev, jerk);
      robot.set_optimizer_params(std::min(120.0f, max_dev + 5.0f), jerk);
      (void)snprintf(toast_buf, toast_buf_len, "TUNING: Max Dev = %.0f mm",
                     std::min(120.0f, max_dev + 5.0f));
      toast_timer = 2.0f;
      return true;
    }
    case lf::sim::GuiAction::kTuneCorridorDown: {
      float max_dev = 0.0f;
      float jerk = 0.0f;
      robot.get_optimizer_params(max_dev, jerk);
      robot.set_optimizer_params(std::max(0.0f, max_dev - 5.0f), jerk);
      (void)snprintf(toast_buf, toast_buf_len, "TUNING: Max Dev = %.0f mm",
                     std::max(0.0f, max_dev - 5.0f));
      toast_timer = 2.0f;
      return true;
    }
    case lf::sim::GuiAction::kTriggerLiveReopt: {
      if (!engine.has_trajectory_data()) {
        (void)snprintf(toast_buf, toast_buf_len,
                       "NOTICE: Run [1] MAP first to record track path before optimizing!");
        toast_timer = 3.5f;
      } else {
        engine.recompute_racing_trajectory(manifest.start_x, manifest.start_y);
        (void)snprintf(toast_buf, toast_buf_len,
                       "LIVE RE-OPTIMIZE: Minimum-Curvature C2 QP Recalculated!");
        toast_timer = 3.0f;
      }
      return true;
    }
    default:
      return false;
  }
}

void handle_gui_action(lf::sim::GuiAction gui_act, lf::sim::SimEngine& engine,
                       lf::sim::TrackManifest& manifest, const std::string& manifest_path,
                       bool& follow_robot, bool& is_user_zoomed, char* toast_buf,
                       size_t toast_buf_len, float& toast_timer) {
  if (handle_tuning_action(gui_act, engine, manifest, toast_buf, toast_buf_len, toast_timer)) {
    return;
  }

  const auto& robot = engine.get_robot();
  auto& state = engine.get_state_mut();
  const float th_rad = manifest.start_theta_deg * (lf::constants::kPi / 180.0f);

  switch (gui_act) {
    case lf::sim::GuiAction::kAutoTune: {
      engine.reset_pose(lf::RobotMode::kAutoTune, false, manifest.start_x, manifest.start_y,
                        th_rad);
      (void)snprintf(toast_buf, toast_buf_len,
                     "AUTO-TUNE ACTIVE: Åström–Hägglund Relay limit cycle running...");
      toast_timer = 3.5f;
      break;
    }
    case lf::sim::GuiAction::kRun1Map: {
      engine.reset_pose(lf::RobotMode::kRun1Map, false, manifest.start_x, manifest.start_y, th_rad);
      (void)snprintf(toast_buf, toast_buf_len, "RUN 1 ACTIVE: Map Scanning & Path Learning");
      toast_timer = 3.0f;
      break;
    }
    case lf::sim::GuiAction::kRun2Sprint: {
      engine.reset_pose(lf::RobotMode::kRun2Sprint, false, manifest.start_x, manifest.start_y,
                        th_rad);
      (void)snprintf(toast_buf, toast_buf_len, "RUN 2 ACTIVE: C2 Racing Line Sprint");
      toast_timer = 3.0f;
      break;
    }
    case lf::sim::GuiAction::kTogglePause: {
      state.is_paused = !state.is_paused;
      if (state.is_paused && state.trail.size() > 10 && !state.has_run1_data &&
          robot.get_mode() == lf::RobotMode::kRun1Map) {
        state.unoptimized_trail.assign(state.trail.begin(), state.trail.end());
        state.run1_time_s = state.sim_time;
        state.has_run1_data = true;
        engine.recompute_racing_trajectory(manifest.start_x, manifest.start_y);
      }
      break;
    }
    case lf::sim::GuiAction::kResetPose: {
      const auto cur_mode = robot.get_mode();
      const auto reset_mode = (cur_mode == lf::RobotMode::kRun2Sprint) ? lf::RobotMode::kRun2Sprint
                                                                       : lf::RobotMode::kRun1Map;
      engine.reset_pose(reset_mode, false, manifest.start_x, manifest.start_y, th_rad);
      state.is_paused = false;
      (void)snprintf(toast_buf, toast_buf_len, "RESET: Returned to Start (%s)",
                     (reset_mode == lf::RobotMode::kRun2Sprint) ? "Sprint" : "Map");
      toast_timer = 2.5f;
      break;
    }
    case lf::sim::GuiAction::kResetAll: {
      engine.reset_pose(lf::RobotMode::kIdle, true, manifest.start_x, manifest.start_y, th_rad);
      break;
    }
    case lf::sim::GuiAction::kToggleFollowCam: {
      follow_robot = !follow_robot;
      if (follow_robot) {
        is_user_zoomed = false;
        state.is_auto_scale_cam = false;
      }
      break;
    }
    case lf::sim::GuiAction::kFitArena: {
      follow_robot = false;
      is_user_zoomed = false;
      state.is_auto_scale_cam = false;
      break;
    }
    case lf::sim::GuiAction::kClearObstacles: {
      engine.set_obstacles({});
      manifest.obstacles.clear();
      engine.recompute_racing_trajectory(manifest.start_x, manifest.start_y);
      break;
    }
    case lf::sim::GuiAction::kCommitC2: {
      if (state.trail.size() > 10) {
        state.unoptimized_trail.assign(state.trail.begin(), state.trail.end());
        state.run1_time_s = state.sim_time;
        state.has_run1_data = true;
        engine.recompute_racing_trajectory(manifest.start_x, manifest.start_y);
        (void)snprintf(toast_buf, toast_buf_len,
                       "OPTIMIZED C2 TRAJECTORY: Minimum-Curvature Solved.");
        toast_timer = 3.0f;
      }
      break;
    }
    case lf::sim::GuiAction::kToggleSetupMode: {
      state.is_setup_mode = !state.is_setup_mode;
      (void)snprintf(toast_buf, toast_buf_len, "SETUP MODE: %s",
                     state.is_setup_mode ? "ACTIVE (Click canvas to edit)" : "OFF");
      toast_timer = 2.5f;
      break;
    }
    case lf::sim::GuiAction::kSetToolSelect: {
      state.setup_tool = lf::sim::SetupTool::kSelect;
      break;
    }
    case lf::sim::GuiAction::kSetToolStart: {
      state.setup_tool = lf::sim::SetupTool::kSetStart;
      (void)snprintf(toast_buf, toast_buf_len, "TOOL: Click on line to set Start Pose");
      toast_timer = 2.5f;
      break;
    }
    case lf::sim::GuiAction::kSetToolFinish: {
      state.setup_tool = lf::sim::SetupTool::kSetFinish;
      (void)snprintf(toast_buf, toast_buf_len, "TOOL: Click on line to set Finish Box");
      toast_timer = 2.5f;
      break;
    }
    case lf::sim::GuiAction::kSetToolAddObs: {
      state.setup_tool = lf::sim::SetupTool::kAddObstacle;
      (void)snprintf(toast_buf, toast_buf_len, "TOOL: Click on canvas to place Obstacle");
      toast_timer = 2.5f;
      break;
    }
    case lf::sim::GuiAction::kDeleteHoveredObs: {
      auto obstacles = engine.get_obstacles();
      if (state.hovered_obstacle_idx >= 0 &&
          state.hovered_obstacle_idx < static_cast<int>(obstacles.size())) {
        obstacles.erase(obstacles.begin() + state.hovered_obstacle_idx);
        engine.set_obstacles(obstacles);
        manifest.obstacles = std::move(obstacles);
        state.hovered_obstacle_idx = -1;
        (void)snprintf(toast_buf, toast_buf_len, "DELETED OBSTACLE");
        toast_timer = 2.0f;
        engine.recompute_racing_trajectory(manifest.start_x, manifest.start_y);
      }
      break;
    }
    case lf::sim::GuiAction::kSaveTrack: {
      manifest.obstacles = engine.get_obstacles();
      if (lf::sim::save_track_manifest(manifest_path, manifest)) {
        (void)snprintf(toast_buf, toast_buf_len, "TRACK SAVED: %s", manifest_path.c_str());
        toast_timer = 3.0f;
      }
      break;
    }
    default:
      break;
  }
}

void render_canvas_hover(Vector2 mouse_pos, float view_w, const Camera2D& camera,
                         const lf::sim::SimRobotState& state, float ui_scale, const Font& font_bold,
                         const Font& font_mono) {
  if (mouse_pos.x >= view_w || state.planned_trajectory.empty()) {
    return;
  }
  const Vector2 mouse_world = GetScreenToWorld2D(mouse_pos, camera);
  float min_d = 9999.0f;
  size_t nearest_idx = 0;
  for (size_t i = 0; i < state.planned_trajectory.size(); ++i) {
    const auto& pt = state.planned_trajectory.at(i);
    const float dx = pt.x_mm - mouse_world.x;
    const float dy = pt.y_mm - mouse_world.y;
    const float d = std::sqrt(dx * dx + dy * dy);
    if (d < min_d) {
      min_d = d;
      nearest_idx = i;
    }
  }
  if (min_d < 45.0f) {
    lf::sim::draw_canvas_hover_telemetry(mouse_pos, state.planned_trajectory.at(nearest_idx),
                                         nearest_idx, ui_scale, font_bold, font_mono);
  }
}

void update_telemetry_ground_station(lf::sim::TelemetryReceiver& receiver,
                                     lf::sim::SimEngine& engine, float dt, int32_t& prev_enc_l,
                                     int32_t& prev_enc_r, bool& is_first_pkt) {
  lf::TelemetryPacket pkt{};
  if (receiver.poll_latest(pkt)) {
    engine.update_from_telemetry(pkt, dt, prev_enc_l, prev_enc_r, is_first_pkt);
  } else {
    engine.get_state_mut().sim_time += dt;
  }
}

void update_frame_timers(float dt, float& toast_timer, lf::sim::SimRobotState& state) {
  if (toast_timer > 0.0f) {
    toast_timer -= dt;
  }
  if (state.key_flash_timer > 0.0f) {
    state.key_flash_timer -= dt;
  }
}

void run_physics_substeps(lf::sim::SimEngine& engine, float dt, float& accumulator, char* toast_msg,
                          size_t toast_size, float& toast_timer) {
  const auto& state = engine.get_state();
  if (state.is_paused || state.is_crashed) {
    return;
  }

  constexpr float kSubStepDt = 0.002f;  // 500 Hz MCU step
  accumulator += dt;
  while (accumulator >= kSubStepDt) {
    const auto step_res = engine.step(kSubStepDt);
    if (step_res.collision) {
      (void)snprintf(toast_msg, toast_size, "CRASH DETECTED! Hit Obstacle #%d -> Emergency Stop.",
                     state.crashed_obstacle_idx + 1);
      toast_timer = 5.0f;
    }
    accumulator -= kSubStepDt;
  }
}

void check_mode_transition_toasts(lf::RobotMode prev_mode, lf::RobotMode cur_mode,
                                  const lf::sim::SimEngine& engine, char* toast_msg,
                                  size_t toast_size, float& toast_timer) {
  const auto& state = engine.get_state();
  if (prev_mode == lf::RobotMode::kAutoTune && cur_mode == lf::RobotMode::kIdle) {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    engine.get_robot().get_line_pid_gains(kp, ki, kd);
    (void)snprintf(toast_msg, toast_size, "AUTO-TUNE SUCCESS: Kp=%.4f, Kd=%.4f. Ready for Run 1.",
                   kp, kd);
    toast_timer = 5.0f;
  } else if (prev_mode == lf::RobotMode::kRun1Map && cur_mode == lf::RobotMode::kIdle) {
    (void)snprintf(toast_msg, toast_size,
                   "RUN 1 COMPLETE: %05.2fs. Saved trail & optimized C2 trajectory!",
                   state.run1_time_s);
    toast_timer = 4.0f;
  } else if (prev_mode == lf::RobotMode::kRun2Sprint && cur_mode == lf::RobotMode::kIdle) {
    const float delta_s = state.run2_time_s - state.run1_time_s;
    const float pct = (state.run1_time_s > 0.01f) ? ((delta_s / state.run1_time_s) * 100.0f) : 0.0f;
    (void)snprintf(toast_msg, toast_size, "RUN 2 COMPLETE: %05.2fs [Delta: %+.2fs (%+.1f%%)]",
                   state.run2_time_s, delta_s, pct);
    toast_timer = 5.0f;
  }
}

void render_simulation_canvas(const lf::sim::SimEngine& engine, const Camera2D& camera,
                              Texture2D track_texture, float view_w, float view_h,
                              const lf::RobotConfig& config, lf::RobotMode cur_mode,
                              const lf::sim::TrackManifest& manifest) {
  const auto& state = engine.get_state();
  BeginScissorMode(0, 0, static_cast<int>(view_w), static_cast<int>(view_h));
  BeginMode2D(camera);

  lf::sim::draw_arena_background(track_texture);
  lf::sim::draw_start_and_finish_zones(manifest.start_x, manifest.start_y, manifest.start_theta_deg,
                                       manifest.finish_box.min_x_mm, manifest.finish_box.max_x_mm,
                                       manifest.finish_box.min_y_mm, manifest.finish_box.max_y_mm);
  lf::sim::draw_trajectory_comparison(
    state.unoptimized_trail, state.planned_trajectory,
    engine.get_robot().get_config().profiler.max_straight_speed_mps);
  lf::sim::draw_robot_trail(state.trail, cur_mode);
  lf::sim::draw_robot_chassis(state.x, state.y, state.theta, engine.get_bar_center_x(),
                              engine.get_bar_center_y(), config.kinematics.wheel_base_m,
                              engine.get_sensor_input(), engine.get_robot().get_telemetry(),
                              config.line_sensor.weights_mm, state.is_crashed);
  const float clearance_r = config.environment.clearance_radius_mm();
  lf::sim::draw_obstacles(engine.get_obstacles(), state.hovered_obstacle_idx, clearance_r,
                          state.crashed_obstacle_idx);
  lf::sim::draw_tof_beams(engine.get_bar_center_x(), engine.get_bar_center_y(),
                          engine.get_left_tof_origin(), engine.get_right_tof_origin(), state.theta,
                          engine.get_tof_hits(), engine.get_sensor_input(),
                          config.obstacle.front_detect_dist_m);

  EndMode2D();
  EndScissorMode();
}

Font load_extended_unicode_font(const char* font_path, int font_size) {
  if (!FileExists(font_path)) {
    return GetFontDefault();
  }

  // Construct codepoints array for full ASCII, Latin Extended (Vietnamese), and technical symbols
  std::vector<int> codepoints;
  codepoints.reserve(1024);

  // 1. Basic Latin & ASCII (32 .. 126)
  for (int cp = 32; cp <= 126; ++cp) {
    codepoints.push_back(cp);
  }

  // 2. Latin-1 Supplement (160 .. 255)
  for (int cp = 160; cp <= 255; ++cp) {
    codepoints.push_back(cp);
  }

  // 3. Latin Extended-A & B (256 .. 591)
  for (int cp = 256; cp <= 591; ++cp) {
    codepoints.push_back(cp);
  }

  // 4. Vietnamese Tone Marks: Latin Extended Additional (0x1EA0 .. 0x1EF9)
  for (int cp = 0x1EA0; cp <= 0x1EF9; ++cp) {
    codepoints.push_back(cp);
  }

  // 5. General Punctuation & Currency (0x2010 .. 0x2027, 0x20AB)
  for (int cp = 0x2010; cp <= 0x2027; ++cp) {
    codepoints.push_back(cp);
  }
  codepoints.push_back(0x20AB);  // ₫

  // 6. Arrows & Mathematical Symbols
  codepoints.push_back(0x2190);  // ←
  codepoints.push_back(0x2191);  // ↑
  codepoints.push_back(0x2192);  // →
  codepoints.push_back(0x2193);  // ↓
  codepoints.push_back(0x00B1);  // ±
  codepoints.push_back(0x00B0);  // °
  codepoints.push_back(0x00B2);  // ²
  codepoints.push_back(0x00B3);  // ³

  Font font =
    LoadFontEx(font_path, font_size, codepoints.data(), static_cast<int>(codepoints.size()));
  SetTextureFilter(font.texture, TEXTURE_FILTER_BILINEAR);
  return font;
}

}  // namespace

int main() {
  InitWindow(lf::sim::kBaseWindowWidth, lf::sim::kBaseWindowHeight,
             "LF-Bot 2026: Autonomous Racing Simulation & Authoring Platform");
  SetWindowState(FLAG_WINDOW_RESIZABLE);
  SetTargetFPS(60);

  const lf::RobotConfig config{};
  lf::sim::SimEngine engine(config);

  const auto manifest_paths = lf::sim::get_available_track_manifests();
  size_t current_manifest_idx = 0;
  lf::sim::TrackManifest current_manifest{};

  if (!lf::sim::load_track_manifest(manifest_paths.at(0), current_manifest)) {
    CloseWindow();
    return 1;
  }

  Texture2D track_texture = LoadTexture(current_manifest.image_path.c_str());
  engine.load_manifest(current_manifest);

  const float track_w = engine.get_track_width_mm();
  const float track_h = engine.get_track_height_mm();

  Font font_bold = GetFontDefault();
  Font font_mono = GetFontDefault();
  bool has_custom_fonts = false;
  const char* font_bold_path = nullptr;
  if (FileExists("/usr/share/fonts/TTF/DejaVuSans-Bold.ttf")) {
    font_bold_path = "/usr/share/fonts/TTF/DejaVuSans-Bold.ttf";
  } else if (FileExists("/usr/share/fonts/TTF/DejaVuSans.ttf")) {
    font_bold_path = "/usr/share/fonts/TTF/DejaVuSans.ttf";
  }

  const char* font_mono_path = nullptr;
  if (FileExists("/usr/share/fonts/TTF/DejaVuSansMono.ttf")) {
    font_mono_path = "/usr/share/fonts/TTF/DejaVuSansMono.ttf";
  }

  if (font_bold_path != nullptr && font_mono_path != nullptr) {
    font_bold = load_extended_unicode_font(font_bold_path, 44);
    font_mono = load_extended_unicode_font(font_mono_path, 38);
    has_custom_fonts = true;
  }

  lf::sim::TelemetryReceiver telemetry_receiver;
  (void)telemetry_receiver.open();

  Camera2D camera{};
  camera.target = Vector2{track_w * 0.5f, track_h * 0.5f};
  camera.offset = Vector2{
    static_cast<float>(GetScreenWidth()) * 0.5f,
    static_cast<float>(GetScreenHeight()) * 0.5f,
  };
  camera.rotation = 0.0f;
  camera.zoom = 0.58f;

  bool follow_robot = false;
  bool is_user_zoomed = false;
  int dragged_obstacle_idx = -1;
  Vector2 drag_offset{};
  std::array<char, 128> toast_msg{};
  float toast_timer = 0.0f;

  int32_t prev_telem_enc_l = 0;
  int32_t prev_telem_enc_r = 0;
  bool is_telem_first_pkt = true;

  lf::RobotMode prev_robot_mode = lf::RobotMode::kIdle;
  float physics_accumulator = 0.0f;

  while (!WindowShouldClose()) {
    const int screen_w = GetScreenWidth();
    const int screen_h = GetScreenHeight();
    const float scale_h = static_cast<float>(screen_h) / 1080.0f;
    const float scale_w = static_cast<float>(screen_w) / 1920.0f;
    const float ui_scale = std::clamp(std::max(scale_h, scale_w * 0.85f), 0.85f, 2.40f);

    // Full-bleed 100% canvas viewport (zero-friction hallmark HUD)
    const auto view_w = static_cast<float>(screen_w);
    const auto view_h = static_cast<float>(screen_h);
    const Rectangle view_rect = {0, 0, view_w, view_h};

    update_camera_viewport(camera, view_rect, ui_scale, follow_robot, is_user_zoomed,
                           engine.get_state(), track_w, track_h);

    const auto hotkey_act =
      poll_mode_hotkeys(engine.get_state_mut(), toast_msg.data(), toast_msg.size(), toast_timer);
    if (hotkey_act != lf::sim::GuiAction::kNone) {
      handle_gui_action(hotkey_act, engine, current_manifest,
                        manifest_paths.at(current_manifest_idx), follow_robot, is_user_zoomed,
                        toast_msg.data(), toast_msg.size(), toast_timer);
    }
    handle_camera_hotkeys(view_rect, camera, ui_scale, engine, follow_robot, is_user_zoomed,
                          track_w, track_h);
    handle_mouse_interactions(view_rect, camera, engine, current_manifest,
                              manifest_paths.at(current_manifest_idx), dragged_obstacle_idx,
                              drag_offset, follow_robot, is_user_zoomed, toast_msg.data(),
                              toast_msg.size(), toast_timer);

    float dt = GetFrameTime();
    if (dt > 0.05f) {
      dt = 0.016f;
    }
    update_frame_timers(dt, toast_timer, engine.get_state_mut());

    auto& state = engine.get_state_mut();
    if (state.is_telemetry_mode) {
      update_telemetry_ground_station(telemetry_receiver, engine, dt, prev_telem_enc_l,
                                      prev_telem_enc_r, is_telem_first_pkt);
    } else {
      is_telem_first_pkt = true;
      run_physics_substeps(engine, dt, physics_accumulator, toast_msg.data(), toast_msg.size(),
                           toast_timer);
    }

    const lf::RobotMode cur_mode = engine.get_robot().get_mode();
    check_mode_transition_toasts(prev_robot_mode, cur_mode, engine, toast_msg.data(),
                                 toast_msg.size(), toast_timer);
    prev_robot_mode = cur_mode;

    BeginDrawing();
    ClearBackground(lf::sim::Theme::kCanvasBg);

    render_simulation_canvas(engine, camera, track_texture, view_w, view_h, config, cur_mode,
                             current_manifest);

    const float hov_angle =
      (state.hovered_obstacle_idx >= 0 &&
       state.hovered_obstacle_idx < static_cast<int>(engine.get_obstacles().size()))
        ? engine.get_obstacles().at(static_cast<size_t>(state.hovered_obstacle_idx)).angle_deg
        : -1.0f;

    // Hallmark Zero-Friction Tactical HUD (top pill + bottom ribbon)
    const auto hud_res = lf::sim::draw_tactical_hud(
      view_w, view_h, ui_scale, engine.get_robot_mut(), engine.get_sensor_input(), state,
      engine.get_obstacles().size(), hov_angle, follow_robot, toast_msg.data(), toast_timer,
      font_bold, font_mono, current_manifest.name.c_str());

    if (hud_res.tactical_changed) {
      engine.recompute_racing_trajectory(current_manifest.start_x, current_manifest.start_y);
    }

    if (hud_res.action == lf::sim::GuiAction::kNextTrack || IsKeyPressed(KEY_M)) {
      const size_t next_idx = (current_manifest_idx + 1) % manifest_paths.size();
      switch_to_track_manifest(next_idx, current_manifest_idx, manifest_paths, current_manifest,
                               engine, track_texture, camera, view_rect, ui_scale, follow_robot,
                               is_user_zoomed, toast_msg.data(), toast_msg.size(), toast_timer);
    } else if (hud_res.action == lf::sim::GuiAction::kPrevTrack) {
      const size_t prev_idx =
        (current_manifest_idx + manifest_paths.size() - 1) % manifest_paths.size();
      switch_to_track_manifest(prev_idx, current_manifest_idx, manifest_paths, current_manifest,
                               engine, track_texture, camera, view_rect, ui_scale, follow_robot,
                               is_user_zoomed, toast_msg.data(), toast_msg.size(), toast_timer);
    } else {
      handle_gui_action(hud_res.action, engine, current_manifest,
                        manifest_paths.at(current_manifest_idx), follow_robot, is_user_zoomed,
                        toast_msg.data(), toast_msg.size(), toast_timer);
    }

    render_canvas_hover(GetMousePosition(), view_w, camera, state, ui_scale, font_bold, font_mono);

    EndDrawing();
  }

  if (has_custom_fonts) {
    UnloadFont(font_bold);
    UnloadFont(font_mono);
  }
  UnloadTexture(track_texture);
  CloseWindow();

  return 0;
}
