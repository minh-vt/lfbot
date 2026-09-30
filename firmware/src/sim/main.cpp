#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

#include "control/trajectory_planner.hpp"
#include "hud.hpp"
#include "math/kinematics.hpp"
#include "raycast.hpp"
#include "raylib.h"
#include "raymath.h"
#include "system/robot_coordinator.hpp"
#include "telemetry_receiver.hpp"
#include "types.hpp"
#include "world.hpp"

namespace {

void recompute_racing_trajectory(const lf::RobotCoordinator& robot,
                                 const std::vector<lf::sim::Obstacle>& obstacles,
                                 lf::sim::SimRobotState& state) {
  if (state.unoptimized_trail.size() < 10) {
    return;
  }

  std::vector<lf::TrajectoryPoint> raw_path;
  raw_path.reserve(state.unoptimized_trail.size());
  for (const auto& pt : state.unoptimized_trail) {
    raw_path.push_back({pt.x, pt.y, robot.get_config().profiler.safe_map_speed_mps, 0.0f});
  }

  std::vector<lf::ObstacleGeometry> obs_geom;
  obs_geom.reserve(obstacles.size());
  for (const auto& obs : obstacles) {
    obs_geom.push_back({obs.pos.x, obs.pos.y, obs.width_mm, obs.height_mm, obs.angle_deg});
  }

  const lf::TrajectoryPlanner planner(robot.get_config().environment, robot.get_config().profiler);
  state.planned_trajectory = planner.generate_racing_line(raw_path, obs_geom);
}

void handle_reset_hotkey(lf::RobotCoordinator& robot, lf::sim::SimRobotState& state,
                         char* toast_msg, size_t toast_size, float& toast_timer) {
  if (!IsKeyPressed(KEY_R)) {
    return;
  }
  state.x = lf::sim::kStartPosX;
  state.y = lf::sim::kStartPosY;
  state.theta = 0.0f;
  state.v = 0.0f;
  state.w = 0.0f;
  state.enc_l = 0;
  state.enc_r = 0;
  state.is_paused = true;
  state.sim_time = 0.0f;
  state.trail.clear();

  if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
    robot.init();
    state.unoptimized_trail.clear();
    state.planned_trajectory.clear();
    state.run1_time_s = 0.0f;
    state.run2_time_s = 0.0f;
    state.has_run1_data = false;
    state.has_run2_data = false;
    (void)snprintf(toast_msg, toast_size, "RESET TOAN BO: Xoa pose va toan bo ban do hoc.");
  } else {
    robot.reset_pose();
    (void)snprintf(toast_msg, toast_size, "RESET POSE: Dat lai vi tri (Giu nguyen ban do)");
  }
  toast_timer = 2.5f;
}

void handle_mode_hotkeys(lf::RobotCoordinator& robot,
                         const std::vector<lf::sim::Obstacle>& obstacles,
                         lf::sim::SimRobotState& state, char* toast_msg, size_t toast_size,
                         float& toast_timer) {
  if (IsKeyPressed(KEY_SPACE)) {
    state.is_paused = !state.is_paused;
    if (state.is_paused && state.trail.size() > 10 && !state.has_run1_data &&
        robot.get_mode() == lf::RobotMode::kRun1Map) {
      state.unoptimized_trail.assign(state.trail.begin(), state.trail.end());
      state.run1_time_s = state.sim_time;
      state.has_run1_data = true;
      recompute_racing_trajectory(robot, obstacles, state);
    }
  }
  if (IsKeyPressed(KEY_F4)) {
    state.is_telemetry_mode = !state.is_telemetry_mode;
    if (state.is_telemetry_mode) {
      state.is_paused = false;
      state.trail.clear();
      state.packets_received = 0;
      (void)snprintf(toast_msg, toast_size,
                     "GROUND STATION: Bat thu UDP (Port 9876). Dang cho ket noi robot...");
      toast_timer = 4.0f;
    } else {
      (void)snprintf(toast_msg, toast_size, "SANDBOX SIM: Da tro ve che do mo phong cuc bo.");
      toast_timer = 3.0f;
    }
  }
  if (IsKeyPressed(KEY_ENTER)) {
    if (state.trail.size() > 10) {
      state.unoptimized_trail.assign(state.trail.begin(), state.trail.end());
      state.run1_time_s = state.sim_time;
      state.has_run1_data = true;
      recompute_racing_trajectory(robot, obstacles, state);
      (void)snprintf(toast_msg, toast_size,
                     "CHOT QUY DAO C2: Da render ngay quy dao dua toi uu tu vet chay!");
      toast_timer = 3.5f;
    } else {
      (void)snprintf(toast_msg, toast_size, "CHUA DU DU LIEU: Robot can chay mot doan de ghi vet.");
      toast_timer = 2.0f;
    }
  }
  if (IsKeyPressed(KEY_ONE)) {
    robot.set_mode(lf::RobotMode::kRun1Map);
    state.is_paused = false;
    state.sim_time = 0.0f;
    state.trail.clear();
    (void)snprintf(toast_msg, toast_size, "KICH HOAT: Run 1 (Map Scanning & Profile Learning)");
    toast_timer = 3.0f;
  }
  if (IsKeyPressed(KEY_TWO)) {
    // Luu lai vet chay thuc te Run 1 neu chua co
    if (state.unoptimized_trail.empty() && state.trail.size() > 10) {
      state.unoptimized_trail.assign(state.trail.begin(), state.trail.end());
      state.run1_time_s = state.sim_time;
      state.has_run1_data = true;
    }
    recompute_racing_trajectory(robot, obstacles, state);

    robot.set_mode(lf::RobotMode::kRun2Sprint);
    state.is_paused = false;
    state.sim_time = 0.0f;
    state.trail.clear();
    if (robot.get_profiler().is_optimized()) {
      (void)snprintf(toast_msg, toast_size, "KICH HOAT: Run 2 (C2 Sprint theo ho so toi uu)");
    } else {
      (void)snprintf(toast_msg, toast_size,
                     "KICH HOAT: Run 2 (Sprint phan xa truc tiep theo V_sprint)");
    }
    toast_timer = 3.0f;
  }
  if (IsKeyPressed(KEY_T)) {
    robot.start_auto_tune();
    state.is_paused = false;
    (void)snprintf(toast_msg, toast_size, "KICH HOAT: Relay Auto-Tuning PID...");
    toast_timer = 3.0f;
  }
  handle_reset_hotkey(robot, state, toast_msg, toast_size, toast_timer);
}

void handle_pid_tuning_hotkeys(lf::RobotCoordinator& robot, char* toast_msg, size_t toast_size,
                               float& toast_timer) {
  float cur_kp = 0.0f;
  float cur_ki = 0.0f;
  float cur_kd = 0.0f;
  robot.get_line_pid_gains(cur_kp, cur_ki, cur_kd);

  if (IsKeyPressed(KEY_U)) {
    robot.set_line_pid_gains(cur_kp + 0.005f, cur_ki, cur_kd);
    robot.get_line_pid_gains(cur_kp, cur_ki, cur_kd);
    (void)snprintf(toast_msg, toast_size, "VI CHINH: Line PID Kp = %.4f", cur_kp);
    toast_timer = 2.5f;
  }
  if (IsKeyPressed(KEY_J)) {
    robot.set_line_pid_gains(cur_kp - 0.005f, cur_ki, cur_kd);
    robot.get_line_pid_gains(cur_kp, cur_ki, cur_kd);
    (void)snprintf(toast_msg, toast_size, "VI CHINH: Line PID Kp = %.4f", cur_kp);
    toast_timer = 2.5f;
  }
  if (IsKeyPressed(KEY_I)) {
    robot.set_line_pid_gains(cur_kp, cur_ki, cur_kd + 0.001f);
    robot.get_line_pid_gains(cur_kp, cur_ki, cur_kd);
    (void)snprintf(toast_msg, toast_size, "VI CHINH: Line PID Kd = %.4f", cur_kd);
    toast_timer = 2.5f;
  }
  if (IsKeyPressed(KEY_K)) {
    robot.set_line_pid_gains(cur_kp, cur_ki, cur_kd - 0.001f);
    robot.get_line_pid_gains(cur_kp, cur_ki, cur_kd);
    (void)snprintf(toast_msg, toast_size, "VI CHINH: Line PID Kd = %.4f", cur_kd);
    toast_timer = 2.5f;
  }
  if (IsKeyPressed(KEY_P)) {
    robot.set_line_pid_gains(cur_kp, cur_ki + 0.0001f, cur_kd);
    robot.get_line_pid_gains(cur_kp, cur_ki, cur_kd);
    (void)snprintf(toast_msg, toast_size, "VI CHINH: Line PID Ki = %.5f", cur_ki);
    toast_timer = 2.5f;
  }
  if (IsKeyPressed(KEY_L)) {
    robot.set_line_pid_gains(cur_kp, cur_ki - 0.0001f, cur_kd);
    robot.get_line_pid_gains(cur_kp, cur_ki, cur_kd);
    (void)snprintf(toast_msg, toast_size, "VI CHINH: Line PID Ki = %.5f", cur_ki);
    toast_timer = 2.5f;
  }
}

void handle_tactical_tuning_hotkeys(lf::RobotCoordinator& robot, char* toast_msg, size_t toast_size,
                                    float& toast_timer) {
  const float cur_v_map = robot.get_config().profiler.safe_map_speed_mps;
  const float cur_v_sprint = robot.get_config().profiler.max_straight_speed_mps;
  const float cur_accel = robot.get_config().profiler.max_accel_mps2;
  float cur_obs_front = 0.0f;
  float cur_obs_clear = 0.0f;
  float cur_obs_len = 0.0f;
  robot.get_obstacle_parameters(cur_obs_front, cur_obs_clear, cur_obs_len);

  if (IsKeyPressed(KEY_Y)) {
    robot.set_obstacle_parameters(cur_obs_front + 0.02f, cur_obs_clear, cur_obs_len);
    robot.get_obstacle_parameters(cur_obs_front, cur_obs_clear, cur_obs_len);
    (void)snprintf(toast_msg, toast_size, "VI CHINH: Khoang cach vat can D_obs = %.2f m",
                   cur_obs_front);
    toast_timer = 2.5f;
  }
  if (IsKeyPressed(KEY_H)) {
    robot.set_obstacle_parameters(cur_obs_front - 0.02f, cur_obs_clear, cur_obs_len);
    robot.get_obstacle_parameters(cur_obs_front, cur_obs_clear, cur_obs_len);
    (void)snprintf(toast_msg, toast_size, "VI CHINH: Khoang cach vat can D_obs = %.2f m",
                   cur_obs_front);
    toast_timer = 2.5f;
  }
  if (IsKeyPressed(KEY_B)) {
    robot.set_obstacle_parameters(cur_obs_front, cur_obs_clear + 0.01f, cur_obs_len);
    robot.get_obstacle_parameters(cur_obs_front, cur_obs_clear, cur_obs_len);
    (void)snprintf(toast_msg, toast_size, "VI CHINH: Do rong ne suon Y_clear = %.2f m",
                   cur_obs_clear);
    toast_timer = 2.5f;
  }
  if (IsKeyPressed(KEY_G)) {
    robot.set_obstacle_parameters(cur_obs_front, cur_obs_clear - 0.01f, cur_obs_len);
    robot.get_obstacle_parameters(cur_obs_front, cur_obs_clear, cur_obs_len);
    (void)snprintf(toast_msg, toast_size, "VI CHINH: Do rong ne suon Y_clear = %.2f m",
                   cur_obs_clear);
    toast_timer = 2.5f;
  }
  if (IsKeyPressed(KEY_N)) {
    robot.set_obstacle_parameters(cur_obs_front, cur_obs_clear, cur_obs_len + 0.05f);
    robot.get_obstacle_parameters(cur_obs_front, cur_obs_clear, cur_obs_len);
    (void)snprintf(toast_msg, toast_size, "VI CHINH: Chieu dai cung ne L_chicane = %.2f m",
                   cur_obs_len);
    toast_timer = 2.5f;
  }
  if (IsKeyPressed(KEY_V)) {
    robot.set_obstacle_parameters(cur_obs_front, cur_obs_clear, cur_obs_len - 0.05f);
    robot.get_obstacle_parameters(cur_obs_front, cur_obs_clear, cur_obs_len);
    (void)snprintf(toast_msg, toast_size, "VI CHINH: Chieu dai cung ne L_chicane = %.2f m",
                   cur_obs_len);
    toast_timer = 2.5f;
  }
  if (IsKeyPressed(KEY_E)) {
    robot.set_tactical_speeds(cur_v_map + 0.10f, cur_v_sprint);
    (void)snprintf(toast_msg, toast_size, "VI CHINH: Map Speed V_map = %.2f m/s",
                   robot.get_config().profiler.safe_map_speed_mps);
    toast_timer = 2.5f;
  }
  if (IsKeyPressed(KEY_D)) {
    robot.set_tactical_speeds(cur_v_map - 0.10f, cur_v_sprint);
    (void)snprintf(toast_msg, toast_size, "VI CHINH: Map Speed V_map = %.2f m/s",
                   robot.get_config().profiler.safe_map_speed_mps);
    toast_timer = 2.5f;
  }
  if (IsKeyPressed(KEY_W)) {
    robot.set_tactical_speeds(cur_v_map, cur_v_sprint + 0.20f);
    (void)snprintf(toast_msg, toast_size, "VI CHINH: Sprint Speed V_sprint = %.2f m/s",
                   robot.get_config().profiler.max_straight_speed_mps);
    toast_timer = 2.5f;
  }
  if (IsKeyPressed(KEY_S)) {
    robot.set_tactical_speeds(cur_v_map, cur_v_sprint - 0.20f);
    (void)snprintf(toast_msg, toast_size, "VI CHINH: Sprint Speed V_sprint = %.2f m/s",
                   robot.get_config().profiler.max_straight_speed_mps);
    toast_timer = 2.5f;
  }
  if (IsKeyPressed(KEY_A)) {
    robot.set_profiler_dynamics(cur_accel + 0.5f, 4.5f, 0.8f);
    (void)snprintf(toast_msg, toast_size, "VI CHINH: Max Accel = %.1f m/s^2",
                   robot.get_config().profiler.max_accel_mps2);
    toast_timer = 2.5f;
  }
  if (IsKeyPressed(KEY_Z)) {
    robot.set_profiler_dynamics(cur_accel - 0.5f, 4.5f, 0.8f);
    (void)snprintf(toast_msg, toast_size, "VI CHINH: Max Accel = %.1f m/s^2",
                   robot.get_config().profiler.max_accel_mps2);
    toast_timer = 2.5f;
  }
}

void handle_tactical_hotkeys(lf::RobotCoordinator& robot,
                             const std::vector<lf::sim::Obstacle>& obstacles,
                             lf::sim::SimRobotState& state, char* toast_msg, size_t toast_size,
                             float& toast_timer) {
  handle_mode_hotkeys(robot, obstacles, state, toast_msg, toast_size, toast_timer);
  handle_pid_tuning_hotkeys(robot, toast_msg, toast_size, toast_timer);
  handle_tactical_tuning_hotkeys(robot, toast_msg, toast_size, toast_timer);
}

void handle_camera_hotkeys(Rectangle view_rect, Camera2D& camera, float ui_scale,
                           bool& follow_robot, bool& is_user_zoomed, lf::sim::SimRobotState& state,
                           char* toast_msg, size_t toast_size, float& toast_timer) {
  if (IsKeyPressed(KEY_C)) {
    follow_robot = !follow_robot;
    state.is_auto_scale_cam = false;
    if (follow_robot) {
      camera.offset = Vector2{view_rect.width * 0.5f, view_rect.height * 0.5f};
      camera.zoom = 1.35f * ui_scale;
      is_user_zoomed = true;
    }
  }
  if (IsKeyPressed(KEY_F)) {
    is_user_zoomed = false;
    follow_robot = false;
    state.is_auto_scale_cam = false;
    camera.target = Vector2{lf::sim::kTrackWidthMm * 0.5f, lf::sim::kTrackHeightMm * 0.5f};
    camera.offset = Vector2{view_rect.width * 0.5f, view_rect.height * 0.5f};
    const float margin = 40.0f * ui_scale;
    camera.zoom = std::min((view_rect.width - 2.0f * margin) / lf::sim::kTrackWidthMm,
                           (view_rect.height - 2.0f * margin) / lf::sim::kTrackHeightMm);
  }
  if (IsKeyPressed(KEY_F3)) {
    state.is_auto_scale_cam = !state.is_auto_scale_cam;
    is_user_zoomed = false;
    follow_robot = false;
    (void)snprintf(
      toast_msg, toast_size, "AUTO-SCALE: %s",
      state.is_auto_scale_cam ? "Kich hoat tu dong can chinh sa ban" : "Tat Auto-Scale");
    toast_timer = 2.5f;
  }
}

void handle_obstacle_mouse_drag(Vector2 mouse_world, std::vector<lf::sim::Obstacle>& obstacles,
                                int& dragged_obstacle_idx, Vector2& drag_offset, char* toast_msg,
                                size_t toast_size, float& toast_timer) {
  if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
    int hit_idx = -1;
    for (int i = static_cast<int>(obstacles.size()) - 1; i >= 0; --i) {
      if (Vector2Distance(mouse_world, obstacles.at(static_cast<size_t>(i)).pos) <= 60.0f) {
        hit_idx = i;
        break;
      }
    }
    if (hit_idx >= 0) {
      dragged_obstacle_idx = hit_idx;
      drag_offset = Vector2Subtract(obstacles.at(static_cast<size_t>(hit_idx)).pos, mouse_world);
    } else {
      obstacles.push_back({mouse_world, 105.0f, 60.0f, 0.0f});
      dragged_obstacle_idx = static_cast<int>(obstacles.size()) - 1;
      drag_offset = Vector2{0.0f, 0.0f};
      (void)snprintf(toast_msg, toast_size, "DAT VAT CAN: #%zu tai (%.0f, %.0f)", obstacles.size(),
                     mouse_world.x, mouse_world.y);
      toast_timer = 2.0f;
    }
  }

  if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && dragged_obstacle_idx >= 0 &&
      dragged_obstacle_idx < static_cast<int>(obstacles.size())) {
    auto& obs = obstacles.at(static_cast<size_t>(dragged_obstacle_idx));
    obs.pos = Vector2Add(mouse_world, drag_offset);
    obs.pos.x = std::clamp(obs.pos.x, 60.0f, lf::sim::kTrackWidthMm - 60.0f);
    obs.pos.y = std::clamp(obs.pos.y, 40.0f, lf::sim::kTrackHeightMm - 40.0f);
  }

  if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
    dragged_obstacle_idx = -1;
  }
}

void handle_obstacle_rotation(lf::sim::SimRobotState& state,
                              std::vector<lf::sim::Obstacle>& obstacles, float wheel,
                              const lf::RobotCoordinator& robot, char* toast_msg, size_t toast_size,
                              float& toast_timer) {
  if (state.hovered_obstacle_idx < 0 ||
      state.hovered_obstacle_idx >= static_cast<int>(obstacles.size())) {
    return;
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
  if (IsKeyPressed(KEY_E)) {
    obs.angle_deg = std::fmod(obs.angle_deg + 15.0f + 360.0f, 360.0f);
    changed = true;
  }

  if (changed) {
    (void)snprintf(toast_msg, toast_size, "XOAY VAT CAN #%d: Goc %.0f°",
                   state.hovered_obstacle_idx + 1, obs.angle_deg);
    toast_timer = 1.5f;
    recompute_racing_trajectory(robot, obstacles, state);
  }
}

void handle_mouse_interactions(Rectangle view_rect, Camera2D& camera, float ui_scale,
                               const lf::RobotCoordinator& robot,
                               std::vector<lf::sim::Obstacle>& obstacles,
                               lf::sim::SimRobotState& state, int& dragged_obstacle_idx,
                               Vector2& drag_offset, bool& follow_robot, bool& is_user_zoomed,
                               char* toast_msg, size_t toast_size, float& toast_timer) {
  handle_camera_hotkeys(view_rect, camera, ui_scale, follow_robot, is_user_zoomed, state, toast_msg,
                        toast_size, toast_timer);

  const Vector2 mouse_pos = GetMousePosition();
  if (!CheckCollisionPointRec(mouse_pos, view_rect)) {
    state.hovered_obstacle_idx = -1;
    return;
  }

  const Vector2 mouse_world = GetScreenToWorld2D(mouse_pos, camera);

  // Kiem tra vat can dang duoc hover boi chuot
  int hit_idx = -1;
  for (int i = static_cast<int>(obstacles.size()) - 1; i >= 0; --i) {
    if (Vector2Distance(mouse_world, obstacles.at(static_cast<size_t>(i)).pos) <= 60.0f) {
      hit_idx = i;
      break;
    }
  }
  state.hovered_obstacle_idx = (dragged_obstacle_idx >= 0) ? dragged_obstacle_idx : hit_idx;

  if (IsKeyPressed(KEY_O)) {
    obstacles.push_back({mouse_world, 105.0f, 60.0f, 0.0f});
    (void)snprintf(toast_msg, toast_size, "DA THEM: Vat can #%zu tai (%.0f, %.0f)",
                   obstacles.size(), mouse_world.x, mouse_world.y);
    toast_timer = 2.5f;
    recompute_racing_trajectory(robot, obstacles, state);
  }

  if (IsKeyPressed(KEY_X)) {
    obstacles.clear();
    (void)snprintf(toast_msg, toast_size, "DA XOA: Toan bo vat can tren sa ban.");
    toast_timer = 2.5f;
    recompute_racing_trajectory(robot, obstacles, state);
  }

  handle_obstacle_mouse_drag(mouse_world, obstacles, dragged_obstacle_idx, drag_offset, toast_msg,
                             toast_size, toast_timer);

  if (IsKeyPressed(KEY_BACKSPACE) || IsKeyPressed(KEY_DELETE)) {
    for (auto it = obstacles.begin(); it != obstacles.end(); ++it) {
      if (Vector2Distance(mouse_world, it->pos) <= 60.0f) {
        obstacles.erase(it);
        (void)snprintf(toast_msg, toast_size, "DA XOA: Vat can duoi con tro chuot.");
        toast_timer = 2.0f;
        recompute_racing_trajectory(robot, obstacles, state);
        break;
      }
    }
  }

  const float wheel = GetMouseWheelMove();
  if (state.hovered_obstacle_idx >= 0) {
    handle_obstacle_rotation(state, obstacles, wheel, robot, toast_msg, toast_size, toast_timer);
  } else if (wheel != 0.0f) {
    is_user_zoomed = true;
    camera.offset = mouse_pos;
    camera.target = mouse_world;
    camera.zoom = std::clamp(camera.zoom + wheel * 0.08f * camera.zoom, 0.20f, 4.50f);
  }

  if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
    is_user_zoomed = true;
    const Vector2 delta = GetMouseDelta();
    camera.target.x -= delta.x / camera.zoom;
    camera.target.y -= delta.y / camera.zoom;
    follow_robot = false;
  }
}

void update_simulation(float dt, lf::RobotCoordinator& robot,
                       lf::DifferentialDriveKinematics& kinematics, const Image& blurred_track,
                       const std::vector<lf::sim::Obstacle>& obstacles,
                       lf::sim::SimRobotState& state, lf::SensorData& in,
                       lf::sim::TofHitPoints& hits, Vector2& l_origin, Vector2& r_origin,
                       float& bar_center_x, float& bar_center_y, lf::RobotMode& prev_mode,
                       char* toast_msg, size_t toast_size, float& toast_timer) {
  in.dt = dt;
  const float cos_th = std::cos(state.theta);
  const float sin_th = std::sin(state.theta);

  const float lookahead_mm = robot.get_config().kinematics.sensor_lookahead_m * 1000.0f;
  bar_center_x = state.x + lookahead_mm * cos_th;
  bar_center_y = state.y + lookahead_mm * sin_th;

  for (int i = 0; i < lf::LineSensorConfig::kSensorCount; ++i) {
    const float offset_mm = robot.get_config().line_sensor.weights_mm.at(static_cast<size_t>(i));
    const float sx = bar_center_x - offset_mm * sin_th;
    const float sy = bar_center_y + offset_mm * cos_th;

    const int px = std::clamp(static_cast<int>(sx), 0, blurred_track.width - 1);
    const int py = std::clamp(static_cast<int>(sy), 0, blurred_track.height - 1);

    const Color c = GetImageColor(blurred_track, px, py);
    const float brightness = static_cast<float>(c.r + c.g + c.b) / (3.0f * 255.0f);
    in.ir_raw.at(static_cast<size_t>(i)) = 1.0f - brightness;
  }

  lf::sim::raycast_all_tof(state.x, state.y, state.theta, bar_center_x, bar_center_y, obstacles, in,
                           hits, l_origin, r_origin);

  in.enc_left_ticks = state.enc_l;
  in.enc_right_ticks = state.enc_r;

  lf::MotorCmd out{};
  if (!state.is_paused) {
    robot.update(in, out);
    state.sim_time += dt;
  }

  // Phat hien ket thuc luot chay Run 1 hoac Run 2 de ghi nhan thoi gian
  if (prev_mode == lf::RobotMode::kAutoTuning && robot.get_mode() == lf::RobotMode::kIdle) {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    robot.get_line_pid_gains(kp, ki, kd);
    (void)snprintf(toast_msg, toast_size, "AUTO-TUNE XONG: Kp=%.4f, Ki=%.5f, Kd=%.4f (Da luu)", kp,
                   ki, kd);
    toast_timer = 5.0f;
  }
  if (prev_mode == lf::RobotMode::kRun1Map && robot.get_mode() == lf::RobotMode::kIdle) {
    state.run1_time_s = state.sim_time;
    state.has_run1_data = true;
    if (state.trail.size() > 10) {
      state.unoptimized_trail.assign(state.trail.begin(), state.trail.end());
      recompute_racing_trajectory(robot, obstacles, state);
    }
    (void)snprintf(toast_msg, toast_size,
                   "HOAN THANH RUN 1: %05.2fs. Da luu vet chay va toi uu quy dao C2!",
                   state.run1_time_s);
    toast_timer = 4.0f;
  }
  if (prev_mode == lf::RobotMode::kRun2Sprint && robot.get_mode() == lf::RobotMode::kIdle) {
    state.run2_time_s = state.sim_time;
    state.has_run2_data = true;
    const float delta_s = state.run2_time_s - state.run1_time_s;
    const float pct = (state.run1_time_s > 0.01f) ? ((delta_s / state.run1_time_s) * 100.0f) : 0.0f;
    (void)snprintf(toast_msg, toast_size, "HOAN THANH RUN 2: %05.2fs [Delta: %+.2fs (%+.1f%%)]",
                   state.run2_time_s, delta_s, pct);
    toast_timer = 5.0f;
  }
  prev_mode = robot.get_mode();

  state.left_pwm = out.left_pwm;
  state.right_pwm = out.right_pwm;

  const float max_speed = robot.get_config().profiler.max_straight_speed_mps;
  float v_l = out.left_pwm * max_speed;
  float v_r = out.right_pwm * max_speed;

  if (out.active_brake) {
    v_l = 0.0f;
    v_r = 0.0f;
  }

  const auto twist = kinematics.forward(v_l, v_r);
  state.v = twist.linear_v_mps;
  state.w = twist.angular_w_radps;

  const float m_per_tick = robot.get_config().kinematics.meters_per_tick();
  state.enc_l += static_cast<int32_t>((v_l * dt) / m_per_tick);
  state.enc_r += static_cast<int32_t>((v_r * dt) / m_per_tick);

  state.x += state.v * cos_th * dt * 1000.0f;
  state.y += state.v * sin_th * dt * 1000.0f;
  state.theta += state.w * dt;

  const float d_sq = state.trail.empty()
                       ? 100.0f
                       : (state.trail.back().x - state.x) * (state.trail.back().x - state.x) +
                           (state.trail.back().y - state.y) * (state.trail.back().y - state.y);
  if (state.trail.empty() || d_sq > 64.0f) {
    state.trail.push_back(Vector2{state.x, state.y});
    if (state.trail.size() > lf::sim::kMaxTrailPoints) {
      state.trail.pop_front();
    }
  }
}

void update_camera_viewport(Camera2D& camera, Rectangle view_rect, float ui_scale,
                            bool follow_robot, bool is_user_zoomed,
                            const lf::sim::SimRobotState& state) {
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
    }
  } else if (!is_user_zoomed && !follow_robot) {
    const float margin = 40.0f * ui_scale;
    const float scale_x = (view_w - 2.0f * margin) / lf::sim::kTrackWidthMm;
    const float scale_y = (view_h - 2.0f * margin) / lf::sim::kTrackHeightMm;
    camera.zoom = std::min(scale_x, scale_y);
    camera.target = Vector2{lf::sim::kTrackWidthMm * 0.5f, lf::sim::kTrackHeightMm * 0.5f};
    camera.offset = Vector2{view_w * 0.5f, view_h * 0.5f};
  } else if (follow_robot) {
    camera.offset = Vector2{view_w * 0.5f, view_h * 0.5f};
    camera.target.x += (state.x - camera.target.x) * 0.12f;
    camera.target.y += (state.y - camera.target.y) * 0.12f;
  }
}

void update_telemetry_ground_station(lf::sim::TelemetryReceiver& telemetry_receiver,
                                     lf::RobotCoordinator& robot, lf::sim::SimRobotState& state,
                                     lf::SensorData& in, float dt, int32_t& prev_telem_enc_l,
                                     int32_t& prev_telem_enc_r, bool& is_telem_first_pkt) {
  lf::TelemetryPacket pkt{};
  if (telemetry_receiver.poll_latest(pkt)) {
    state.packets_received++;
    state.left_pwm = pkt.left_pwm;
    state.right_pwm = pkt.right_pwm;

    in.ir_raw = pkt.ir_raw;
    in.tof_front_m = pkt.tof_front_m;
    in.tof_left_m = pkt.tof_left_m;
    in.tof_right_m = pkt.tof_right_m;
    in.enc_left_ticks = pkt.enc_left_ticks;
    in.enc_right_ticks = pkt.enc_right_ticks;

    auto& tel = robot.get_telemetry_mut();
    tel.centroid_pos_mm = pkt.centroid_pos_mm;
    tel.line_lost = (pkt.line_lost != 0);
    tel.battery_v = pkt.battery_voltage_v;
    tel.mode = static_cast<lf::RobotMode>(pkt.mode);
    tel.fsm_state = pkt.fsm_state;

    if (is_telem_first_pkt) {
      prev_telem_enc_l = pkt.enc_left_ticks;
      prev_telem_enc_r = pkt.enc_right_ticks;
      is_telem_first_pkt = false;
    } else {
      const int32_t d_l = pkt.enc_left_ticks - prev_telem_enc_l;
      const int32_t d_r = pkt.enc_right_ticks - prev_telem_enc_r;
      prev_telem_enc_l = pkt.enc_left_ticks;
      prev_telem_enc_r = pkt.enc_right_ticks;

      const float m_per_tick = robot.get_config().kinematics.meters_per_tick();
      const float dist_l = static_cast<float>(d_l) * m_per_tick;
      const float dist_r = static_cast<float>(d_r) * m_per_tick;
      const float ds_m = 0.5f * (dist_l + dist_r);
      const float d_th = (dist_r - dist_l) / robot.get_config().kinematics.wheel_base_m;

      const float mid_th = state.theta + 0.5f * d_th;
      state.x += ds_m * std::cos(mid_th) * 1000.0f;
      state.y += ds_m * std::sin(mid_th) * 1000.0f;
      state.theta += d_th;
      state.v = (dt > 0.0001f) ? (ds_m / dt) : 0.0f;
      tel.current_velocity_mps = state.v;

      const float d_sq = state.trail.empty()
                           ? 100.0f
                           : (state.trail.back().x - state.x) * (state.trail.back().x - state.x) +
                               (state.trail.back().y - state.y) * (state.trail.back().y - state.y);
      if (d_sq > 25.0f) {
        state.trail.push_back(Vector2{state.x, state.y});
        if (state.trail.size() > lf::sim::kMaxTrailPoints) {
          state.trail.pop_front();
        }
      }
    }
  }
  state.sim_time += dt;
}

}  // namespace

int main() {
  SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
  InitWindow(lf::sim::kBaseWindowWidth, lf::sim::kBaseWindowHeight,
             "LF-Bot 2026 - Tactical Autonomous Simulator (Raylib SSoT)");
  SetTargetFPS(60);

  Font font_bold = GetFontDefault();
  Font font_mono = GetFontDefault();
  bool has_custom_fonts = false;
  if (FileExists("/usr/share/fonts/liberation/LiberationSans-Bold.ttf")) {
    font_bold = LoadFontEx("/usr/share/fonts/liberation/LiberationSans-Bold.ttf", 80, nullptr, 0);
    SetTextureFilter(font_bold.texture, TEXTURE_FILTER_BILINEAR);
    has_custom_fonts = true;
  }
  if (FileExists("/usr/share/fonts/liberation/LiberationMono-Bold.ttf")) {
    font_mono = LoadFontEx("/usr/share/fonts/liberation/LiberationMono-Bold.ttf", 72, nullptr, 0);
    SetTextureFilter(font_mono.texture, TEXTURE_FILTER_BILINEAR);
    has_custom_fonts = true;
  }

  const lf::RobotConfig config{};
  lf::RobotCoordinator robot(config);
  robot.init();

  lf::DifferentialDriveKinematics kinematics(config.kinematics.wheel_base_m);

  lf::sim::SimRobotState state{};
  std::vector<lf::sim::Obstacle> obstacles = {
    {{1100.0f, 300.0f}, 105.0f, 60.0f, 0.0f},
  };

  Camera2D camera{};
  camera.target = Vector2{lf::sim::kTrackWidthMm * 0.5f, lf::sim::kTrackHeightMm * 0.5f};
  camera.offset =
    Vector2{(static_cast<float>(GetScreenWidth()) - lf::sim::kBaseSidebarWidth) * 0.5f,
            static_cast<float>(GetScreenHeight()) * 0.5f};
  camera.rotation = 0.0f;
  camera.zoom = 0.58f;

  bool follow_robot = false;
  bool is_user_zoomed = false;
  int dragged_obstacle_idx = -1;
  Vector2 drag_offset = {0.0f, 0.0f};

  std::array<char, 128> toast_msg{};
  (void)snprintf(toast_msg.data(), toast_msg.size(),
                 "CHAO MUNG: Bam [1] de hoc ban do, [2] de chay Sprint quy dao toi uu!");
  float toast_timer = 4.0f;

  Image raw_track = LoadImage("assets/track.png");
  if (raw_track.data == nullptr) {
    TraceLog(LOG_ERROR, "Khong tim thay assets/track.png! Hay chay generate_track.py.");
    CloseWindow();
    return 1;
  }

  ImageFormat(&raw_track, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);
  Image blurred_track = ImageCopy(raw_track);
  ImageBlurGaussian(&blurred_track, 2);
  const Texture2D track_texture = LoadTextureFromImage(raw_track);

  lf::RobotMode prev_robot_mode = robot.get_mode();

  lf::sim::TelemetryReceiver telemetry_receiver;
  (void)telemetry_receiver.open(lf::kDefaultTelemetryPort);
  int32_t prev_telem_enc_l = 0;
  int32_t prev_telem_enc_r = 0;
  bool is_telem_first_pkt = true;

  while (!WindowShouldClose()) {
    const int screen_w = GetScreenWidth();
    const int screen_h = GetScreenHeight();
    const float ui_scale = std::clamp(static_cast<float>(screen_h) / 1080.0f, 0.65f, 1.75f);
    const float sb_w = lf::sim::kBaseSidebarWidth * ui_scale;
    const float sb_x = static_cast<float>(screen_w) - sb_w;
    const float view_w = sb_x;
    const auto view_h = static_cast<float>(screen_h);
    const Rectangle view_rect = {0, 0, view_w, view_h};

    update_camera_viewport(camera, view_rect, ui_scale, follow_robot, is_user_zoomed, state);

    handle_tactical_hotkeys(robot, obstacles, state, toast_msg.data(), toast_msg.size(),
                            toast_timer);
    handle_mouse_interactions(view_rect, camera, ui_scale, robot, obstacles, state,
                              dragged_obstacle_idx, drag_offset, follow_robot, is_user_zoomed,
                              toast_msg.data(), toast_msg.size(), toast_timer);

    float dt = GetFrameTime();
    if (dt > 0.05f) {
      dt = 0.016f;
    }

    lf::SensorData in{};
    lf::sim::TofHitPoints hits{};
    Vector2 l_origin{};
    Vector2 r_origin{};
    float bar_center_x = 0.0f;
    float bar_center_y = 0.0f;

    if (state.is_telemetry_mode) {
      update_telemetry_ground_station(telemetry_receiver, robot, state, in, dt, prev_telem_enc_l,
                                      prev_telem_enc_r, is_telem_first_pkt);
      const float lookahead_mm = robot.get_config().kinematics.sensor_lookahead_m * 1000.0f;
      bar_center_x = state.x + lookahead_mm * std::cos(state.theta);
      bar_center_y = state.y + lookahead_mm * std::sin(state.theta);
    } else {
      is_telem_first_pkt = true;
      update_simulation(dt, robot, kinematics, blurred_track, obstacles, state, in, hits, l_origin,
                        r_origin, bar_center_x, bar_center_y, prev_robot_mode, toast_msg.data(),
                        toast_msg.size(), toast_timer);
    }

    // Render Scene
    BeginDrawing();
    ClearBackground(Color{13, 17, 23, 255});

    BeginScissorMode(0, 0, static_cast<int>(view_w), static_cast<int>(view_h));
    BeginMode2D(camera);

    lf::sim::draw_arena_background(track_texture);
    lf::sim::draw_trajectory_comparison(state.unoptimized_trail, state.planned_trajectory,
                                        robot.get_config().profiler.max_straight_speed_mps);
    lf::sim::draw_robot_trail(state.trail, robot.get_mode());
    lf::sim::draw_robot_chassis(state.x, state.y, state.theta, bar_center_x, bar_center_y,
                                robot.get_config().kinematics.wheel_base_m, in,
                                robot.get_telemetry(), robot.get_config().line_sensor.weights_mm);
    const float clearance_r = robot.get_config().environment.clearance_radius_mm();
    lf::sim::draw_obstacles(obstacles, state.hovered_obstacle_idx, clearance_r);
    lf::sim::draw_tof_beams(bar_center_x, bar_center_y, l_origin, r_origin, state.theta, hits, in);

    EndMode2D();
    EndScissorMode();

    // Render HUD Sidebar
    lf::sim::draw_sidebar_background(sb_x, sb_w, screen_h, ui_scale);

    const float pad_x = 24.0f * ui_scale;
    const float card_w = sb_w - 2.0f * pad_x;
    const float half_w = (card_w - 12.0f * ui_scale) * 0.5f;

    lf::sim::draw_header_card(sb_x, pad_x, card_w, ui_scale, state.sim_time, state.is_paused,
                              robot.get_mode(), state.is_telemetry_mode, state.packets_received,
                              font_bold, font_mono);
    lf::sim::draw_telemetry_card(sb_x, pad_x, half_w, ui_scale, state.v,
                                 robot.get_config().profiler.max_straight_speed_mps,
                                 robot.get_telemetry(), font_bold);
    lf::sim::draw_centroid_card(sb_x, pad_x, card_w, ui_scale, robot.get_telemetry(), font_bold);
    lf::sim::draw_sensors_card(sb_x, pad_x, card_w, ui_scale, in, font_bold);
    lf::sim::draw_motor_pwm_card(sb_x, pad_x, card_w, ui_scale, state.left_pwm, state.right_pwm,
                                 robot.get_telemetry(), font_bold, font_mono);

    const float hov_angle =
      (state.hovered_obstacle_idx >= 0 &&
       state.hovered_obstacle_idx < static_cast<int>(obstacles.size()))
        ? obstacles.at(static_cast<size_t>(state.hovered_obstacle_idx)).angle_deg
        : -1.0f;
    lf::sim::draw_tof_card(sb_x, pad_x, card_w, ui_scale, in, obstacles.size(), hov_angle,
                           font_bold, font_mono);
    lf::sim::draw_lap_comparison_card(sb_x, pad_x, card_w, ui_scale, state, font_bold, font_mono);
    lf::sim::draw_tactical_card(sb_x, pad_x, card_w, ui_scale, robot, font_bold, font_mono);

    if (toast_timer > 0.0f) {
      lf::sim::draw_toast_notification(sb_x, pad_x, card_w, ui_scale, toast_msg.data(), font_bold);
    }

    lf::sim::draw_controls_card(sb_x, pad_x, card_w, view_h, ui_scale, follow_robot, font_bold);

    EndDrawing();
  }

  if (has_custom_fonts) {
    UnloadFont(font_bold);
    UnloadFont(font_mono);
  }
  UnloadTexture(track_texture);
  UnloadImage(raw_track);
  UnloadImage(blurred_track);
  CloseWindow();

  return 0;
}
