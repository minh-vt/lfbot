#include "world.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "config/robot_config.hpp"
#include "theme.hpp"

namespace lf::sim {

void draw_rotated_box(Vector2 center, float width, float height, float angle_deg, Color fill,
                      Color border, float border_thick) {
  const Rectangle rect = {center.x, center.y, width, height};
  const Vector2 origin = {width * 0.5f, height * 0.5f};
  DrawRectanglePro(rect, origin, angle_deg, fill);

  if (border_thick <= 0.0f) {
    return;
  }

  const float angle_rad = angle_deg * DEG2RAD;
  const float cos_a = std::cos(angle_rad);
  const float sin_a = std::sin(angle_rad);

  const float hw = width * 0.5f;
  const float hh = height * 0.5f;

  const Vector2 p1 = {center.x - hw * cos_a + hh * sin_a, center.y - hw * sin_a - hh * cos_a};
  const Vector2 p2 = {center.x + hw * cos_a + hh * sin_a, center.y + hw * sin_a - hh * cos_a};
  const Vector2 p3 = {center.x + hw * cos_a - hh * sin_a, center.y + hw * sin_a + hh * cos_a};
  const Vector2 p4 = {center.x - hw * cos_a - hh * sin_a, center.y - hw * sin_a + hh * cos_a};

  DrawLineEx(p1, p2, border_thick, border);
  DrawLineEx(p2, p3, border_thick, border);
  DrawLineEx(p3, p4, border_thick, border);
  DrawLineEx(p4, p1, border_thick, border);
}

void draw_hazard_obstacle(const Obstacle& obs, bool is_hovered, float clearance_radius_mm) {
  // Circumscribed safety clearance circle R_clearance preventing collisions at any orientation
  const Color clearance_col = is_hovered ? ColorAlpha(Color{251, 191, 36, 255}, 0.18f)
                                         : ColorAlpha(Color{148, 163, 184, 255}, 0.08f);
  const Color ring_col = is_hovered ? Color{251, 191, 36, 180} : Color{148, 163, 184, 90};
  DrawCircle(static_cast<int>(obs.pos.x), static_cast<int>(obs.pos.y), clearance_radius_mm,
             clearance_col);
  DrawCircleLines(static_cast<int>(obs.pos.x), static_cast<int>(obs.pos.y), clearance_radius_mm,
                  ring_col);

  // Bong do hop vat can
  draw_rotated_box(Vector2{obs.pos.x + 4.0f, obs.pos.y + 4.0f}, obs.width_mm, obs.height_mm,
                   obs.angle_deg, ColorAlpha(BLACK, 0.28f), BLANK, 0.0f);

  // Hop vat can chinh
  const Color border_col = is_hovered ? Color{52, 211, 153, 255} : Color{245, 158, 11, 255};
  draw_rotated_box(obs.pos, obs.width_mm, obs.height_mm, obs.angle_deg, Color{30, 41, 59, 255},
                   border_col, is_hovered ? 3.5f : 2.5f);

  // Tam truc va chi so goc xoay
  DrawCircle(static_cast<int>(obs.pos.x), static_cast<int>(obs.pos.y), 3.5f, border_col);

  std::array<char, 16> angle_str{};
  (void)snprintf(angle_str.data(), angle_str.size(), "%.0f°", obs.angle_deg);
  DrawText(angle_str.data(), static_cast<int>(obs.pos.x - 12.0f),
           static_cast<int>(obs.pos.y - 8.0f), 12,
           is_hovered ? Color{52, 211, 153, 255} : Color{245, 158, 11, 220});
}

void draw_arena_background(Texture2D track_texture) {
  const auto track_w = static_cast<float>(track_texture.width);
  const auto track_h = static_cast<float>(track_texture.height);
  DrawRectangle(0, 0, static_cast<int>(track_w), static_cast<int>(track_h), Theme::kArenaBg);
  for (int gx = 0; gx <= static_cast<int>(track_w); gx += 200) {
    DrawLine(gx, 0, gx, static_cast<int>(track_h), Theme::kArenaGrid);
  }
  for (int gy = 0; gy <= static_cast<int>(track_h); gy += 200) {
    DrawLine(0, gy, static_cast<int>(track_w), gy, Theme::kArenaGrid);
  }

  DrawTexture(track_texture, 0, 0, WHITE);
  DrawRectangleLinesEx(Rectangle{0, 0, track_w, track_h}, 4.0f, Theme::kArenaBorder);
}

void draw_robot_trail(const std::deque<Vector2>& trail, lf::RobotMode mode) {
  const Color trail_color =
    (mode == lf::RobotMode::kRun2Sprint) ? Color{251, 191, 36, 180} : Color{56, 189, 248, 160};
  for (size_t i = 1; i < trail.size(); ++i) {
    DrawLineEx(trail.at(i - 1), trail.at(i), 3.0f, trail_color);
  }
}

void draw_trajectory_comparison(const std::vector<Vector2>& unoptimized_trail,
                                const std::vector<lf::TrajectoryPoint>& planned_trajectory,
                                float max_speed_mps) {
  // 1. Vet chay Run 1 thuc te chua toi uu (duong net dut mau cyan kem cham tron)
  if (unoptimized_trail.size() > 1) {
    const Color trail_col = ColorAlpha(Color{56, 189, 248, 255}, 0.55f);
    for (size_t i = 1; i < unoptimized_trail.size(); ++i) {
      if ((i % 5) < 3) {
        DrawLineEx(unoptimized_trail.at(i - 1), unoptimized_trail.at(i), 2.5f, trail_col);
      }
    }
  }

  // 2. Quy dao dua Run 2 toi uu (ruy-bang chuyen mau ho phach -> ngoc luc bao)
  if (planned_trajectory.size() > 1) {
    for (size_t i = 1; i < planned_trajectory.size(); ++i) {
      const auto& p1 = planned_trajectory.at(i - 1);
      const auto& p2 = planned_trajectory.at(i);

      const float v_min = 0.80f;
      const float v_span = std::max(0.10f, max_speed_mps - v_min);
      const float ratio = std::clamp((p2.target_speed_mps - v_min) / v_span, 0.0f, 1.0f);

      // Amber (v = 0.8 m/s): {245, 158, 11}, Emerald (v = 2.5 m/s): {16, 185, 129}
      const auto r = static_cast<unsigned char>(245.0f + (16.0f - 245.0f) * ratio);
      const auto g = static_cast<unsigned char>(158.0f + (185.0f - 158.0f) * ratio);
      const auto b = static_cast<unsigned char>(11.0f + (129.0f - 11.0f) * ratio);
      const Color seg_col = Color{r, g, b, 235};

      DrawLineEx(Vector2{p1.x_mm, p1.y_mm}, Vector2{p2.x_mm, p2.y_mm}, 4.0f, seg_col);
    }
  }
}

void draw_robot_chassis(float robot_x, float robot_y, float robot_theta, float bar_center_x,
                        float bar_center_y, float wheel_base_m, const lf::SensorData& in,
                        const lf::Telemetry& tel,
                        const std::array<float, lf::LineSensorConfig::kSensorCount>& weights_mm,
                        bool is_crashed) {
  const float cos_th = std::cos(robot_theta);
  const float sin_th = std::sin(robot_theta);

  // Khung xe (Chassis highlighted red if crashed)
  const Color chassis_fill = is_crashed ? Color{127, 29, 29, 240} : Color{26, 32, 44, 230};
  const Color chassis_border = is_crashed ? Color{239, 68, 68, 255} : Color{56, 189, 248, 255};
  draw_rotated_box(Vector2{robot_x, robot_y}, kRobotLengthMm, kRobotWidthMm, robot_theta * RAD2DEG,
                   chassis_fill, chassis_border, is_crashed ? 3.5f : 2.0f);

  // Mui ten chi huong mui xe
  const Color fwd_col = is_crashed ? Color{239, 68, 68, 240} : Color{56, 189, 248, 220};
  const Vector2 fwd_tip = {
    robot_x + (kRobotLengthMm * 0.46f) * cos_th,
    robot_y + (kRobotLengthMm * 0.46f) * sin_th,
  };
  const Vector2 fwd_l = {
    robot_x + (kRobotLengthMm * 0.18f) * cos_th - 16.0f * sin_th,
    robot_y + (kRobotLengthMm * 0.18f) * sin_th + 16.0f * cos_th,
  };
  const Vector2 fwd_r = {
    robot_x + (kRobotLengthMm * 0.18f) * cos_th + 16.0f * sin_th,
    robot_y + (kRobotLengthMm * 0.18f) * sin_th - 16.0f * cos_th,
  };
  DrawTriangle(fwd_tip, fwd_l, fwd_r, fwd_col);

  // 2 Banh xe chu dong vi sai
  const float half_wb = wheel_base_m * 500.0f;
  const Vector2 wl_pos = {robot_x - half_wb * (-sin_th), robot_y - half_wb * cos_th};
  const Vector2 wr_pos = {robot_x + half_wb * (-sin_th), robot_y + half_wb * cos_th};
  draw_rotated_box(wl_pos, kWheelLengthMm, kWheelWidthMm, robot_theta * RAD2DEG,
                   Color{15, 23, 42, 255}, Color{148, 163, 184, 255}, 1.5f);
  draw_rotated_box(wr_pos, kWheelLengthMm, kWheelWidthMm, robot_theta * RAD2DEG,
                   Color{15, 23, 42, 255}, Color{148, 163, 184, 255}, 1.5f);

  // Thanh can cam bien & 8 mat doc quang Analog
  DrawLineEx(Vector2{robot_x, robot_y}, Vector2{bar_center_x, bar_center_y}, 3.0f,
             Color{100, 116, 139, 220});
  const float bar_span = 36.0f;
  DrawLineEx(Vector2{bar_center_x - bar_span * (-sin_th), bar_center_y - bar_span * cos_th},
             Vector2{bar_center_x + bar_span * (-sin_th), bar_center_y + bar_span * cos_th}, 4.0f,
             Color{30, 41, 59, 255});

  for (size_t i = 0; i < lf::LineSensorConfig::kSensorCount; ++i) {
    const float offset = weights_mm.at(i);
    const float sx = bar_center_x - offset * sin_th;
    const float sy = bar_center_y + offset * cos_th;
    const float val = in.ir_raw.at(i);

    const Color eye_col = (val > 0.15f) ? ColorAlpha(Color{52, 211, 153, 255}, 0.35f + 0.65f * val)
                                        : Color{71, 85, 105, 180};
    DrawCircle(static_cast<int>(sx), static_cast<int>(sy), 4.5f, eye_col);
    DrawCircleLines(static_cast<int>(sx), static_cast<int>(sy), 4.5f, Color{15, 23, 42, 255});
  }

  // Diem tam Centroid tinh toan
  if (!tel.line_lost) {
    const float cx = bar_center_x - tel.centroid_pos_mm * sin_th;
    const float cy = bar_center_y + tel.centroid_pos_mm * cos_th;
    DrawCircle(static_cast<int>(cx), static_cast<int>(cy), 3.5f, Color{251, 191, 36, 255});
  }
}

void draw_obstacles(const std::vector<Obstacle>& obstacles, int hovered_idx,
                    float clearance_radius_mm, int crashed_idx) {
  for (size_t i = 0; i < obstacles.size(); ++i) {
    const bool is_hovered = (static_cast<int>(i) == hovered_idx);
    const bool is_crashed = (static_cast<int>(i) == crashed_idx);
    draw_hazard_obstacle(obstacles.at(i), is_hovered || is_crashed, clearance_radius_mm);
    if (is_crashed) {
      draw_rotated_box(obstacles.at(i).pos, obstacles.at(i).width_mm + 10.0f,
                       obstacles.at(i).height_mm + 10.0f, obstacles.at(i).angle_deg,
                       ColorAlpha(Color{239, 68, 68, 255}, 0.35f), Color{239, 68, 68, 255}, 3.0f);
    }
  }
}

void draw_tof_beams(float bar_center_x, float bar_center_y, Vector2 l_origin, Vector2 r_origin,
                    float robot_theta, const TofHitPoints& hits, const lf::SensorData& in,
                    float d_obs_m) {
  (void)in;
  draw_rotated_box(Vector2{bar_center_x, bar_center_y}, 14.0f, 10.0f, robot_theta * RAD2DEG,
                   Color{15, 23, 42, 255}, Color{56, 189, 248, 255}, 1.0f);
  draw_rotated_box(l_origin, 10.0f, 12.0f, (robot_theta - 1.5707963f) * RAD2DEG,
                   Color{15, 23, 42, 255}, Color{251, 191, 36, 255}, 1.0f);
  draw_rotated_box(r_origin, 10.0f, 12.0f, (robot_theta + 1.5707963f) * RAD2DEG,
                   Color{15, 23, 42, 255}, Color{192, 132, 252, 255}, 1.0f);

  constexpr float kHalfFovDeg = 12.5f;

  // 1. Front ToF 25-degree Raycone (dynamic active sensing cone governed by D_obs)
  const float d_obs_mm = std::max(80.0f, d_obs_m * 1000.0f);
  const float front_deg = robot_theta * RAD2DEG;
  const Vector2 front_origin = {bar_center_x, bar_center_y};

  if (hits.front.has_hit) {
    const float hit_dist = std::clamp(hits.front.distance_mm, 10.0f, d_obs_mm);
    const Color alert_col = Color{239, 68, 68, 255};  // Red Alert

    // Truncated illuminated active beam stopping at obstacle face
    DrawCircleSector(front_origin, hit_dist, front_deg - kHalfFovDeg, front_deg + kHalfFovDeg, 24,
                     ColorAlpha(alert_col, 0.28f));
    DrawCircleSectorLines(front_origin, hit_dist, front_deg - kHalfFovDeg, front_deg + kHalfFovDeg,
                          24, ColorAlpha(alert_col, 0.90f));

    // Ghost boundary indicating occluded detection zone up to D_obs
    DrawCircleSectorLines(front_origin, d_obs_mm, front_deg - kHalfFovDeg, front_deg + kHalfFovDeg,
                          24, ColorAlpha(Color{148, 163, 184, 255}, 0.22f));

    // Internal optical sightline to exact contact point on obstacle
    DrawLineEx(front_origin, hits.front.to_vec2(), 2.5f, alert_col);

    // Target radar ring on obstacle surface
    DrawCircle(static_cast<int>(hits.front.x), static_cast<int>(hits.front.y), 5.0f, alert_col);
    DrawCircleLines(static_cast<int>(hits.front.x), static_cast<int>(hits.front.y), 9.0f,
                    ColorAlpha(alert_col, 0.85f));
  } else {
    // Open scan raycone freely illuminating up to D_obs (no obstacle in cone)
    const Color scan_col = Color{56, 189, 248, 255};  // High-tech Cyan
    DrawCircleSector(front_origin, d_obs_mm, front_deg - kHalfFovDeg, front_deg + kHalfFovDeg, 24,
                     ColorAlpha(scan_col, 0.09f));
    DrawCircleSectorLines(front_origin, d_obs_mm, front_deg - kHalfFovDeg, front_deg + kHalfFovDeg,
                          24, ColorAlpha(scan_col, 0.45f));
  }

  // 2. Left ToF 25-degree Raycone (Side flank scanning, 200mm range)
  constexpr float kSideRangeMm = 200.0f;
  const float left_deg = (robot_theta - 1.5707963f) * RAD2DEG;
  const Color l_col = Color{251, 191, 36, 255};  // Amber

  if (hits.left.has_hit) {
    const float hit_dist = std::clamp(hits.left.distance_mm, 10.0f, kSideRangeMm);
    DrawCircleSector(l_origin, hit_dist, left_deg - kHalfFovDeg, left_deg + kHalfFovDeg, 20,
                     ColorAlpha(l_col, 0.30f));
    DrawCircleSectorLines(l_origin, hit_dist, left_deg - kHalfFovDeg, left_deg + kHalfFovDeg, 20,
                          ColorAlpha(l_col, 0.90f));
    DrawCircleSectorLines(l_origin, kSideRangeMm, left_deg - kHalfFovDeg, left_deg + kHalfFovDeg,
                          20, ColorAlpha(Color{148, 163, 184, 255}, 0.20f));
    DrawLineEx(l_origin, hits.left.to_vec2(), 2.0f, l_col);
    DrawCircle(static_cast<int>(hits.left.x), static_cast<int>(hits.left.y), 4.0f, l_col);
  } else {
    DrawCircleSector(l_origin, kSideRangeMm, left_deg - kHalfFovDeg, left_deg + kHalfFovDeg, 20,
                     ColorAlpha(l_col, 0.08f));
    DrawCircleSectorLines(l_origin, kSideRangeMm, left_deg - kHalfFovDeg, left_deg + kHalfFovDeg,
                          20, ColorAlpha(l_col, 0.35f));
  }

  // 3. Right ToF 25-degree Raycone (Side flank scanning, 200mm range)
  const float right_deg = (robot_theta + 1.5707963f) * RAD2DEG;
  const Color r_col = Color{192, 132, 252, 255};  // Purple

  if (hits.right.has_hit) {
    const float hit_dist = std::clamp(hits.right.distance_mm, 10.0f, kSideRangeMm);
    DrawCircleSector(r_origin, hit_dist, right_deg - kHalfFovDeg, right_deg + kHalfFovDeg, 20,
                     ColorAlpha(r_col, 0.30f));
    DrawCircleSectorLines(r_origin, hit_dist, right_deg - kHalfFovDeg, right_deg + kHalfFovDeg, 20,
                          ColorAlpha(r_col, 0.90f));
    DrawCircleSectorLines(r_origin, kSideRangeMm, right_deg - kHalfFovDeg, right_deg + kHalfFovDeg,
                          20, ColorAlpha(Color{148, 163, 184, 255}, 0.20f));
    DrawLineEx(r_origin, hits.right.to_vec2(), 2.0f, r_col);
    DrawCircle(static_cast<int>(hits.right.x), static_cast<int>(hits.right.y), 4.0f, r_col);
  } else {
    DrawCircleSector(r_origin, kSideRangeMm, right_deg - kHalfFovDeg, right_deg + kHalfFovDeg, 20,
                     ColorAlpha(r_col, 0.08f));
    DrawCircleSectorLines(r_origin, kSideRangeMm, right_deg - kHalfFovDeg, right_deg + kHalfFovDeg,
                          20, ColorAlpha(r_col, 0.35f));
  }
}

void draw_start_and_finish_zones(float start_x, float start_y, float start_theta_deg,
                                 float fin_min_x, float fin_max_x, float fin_min_y,
                                 float fin_max_y) {
  // 1. Finish Box
  const float fb_w = fin_max_x - fin_min_x;
  const float fb_h = fin_max_y - fin_min_y;
  DrawRectangleRounded(Rectangle{fin_min_x, fin_min_y, fb_w, fb_h}, 0.15f, 4,
                       ColorAlpha(Color{16, 185, 129, 255}, 0.18f));
  DrawRectangleRoundedLines(Rectangle{fin_min_x, fin_min_y, fb_w, fb_h}, 0.15f, 4,
                            Color{16, 185, 129, 240});
  DrawText("FINISH BOX", static_cast<int>(fin_min_x + 8.0f), static_cast<int>(fin_min_y + 8.0f), 16,
           Color{16, 185, 129, 255});

  // 2. Start Pose Marker
  const float th_rad = start_theta_deg * DEG2RAD;
  DrawCircleLines(static_cast<int>(start_x), static_cast<int>(start_y), 24.0f,
                  Color{56, 189, 248, 200});
  DrawCircle(static_cast<int>(start_x), static_cast<int>(start_y), 5.0f, Color{56, 189, 248, 255});
  DrawLineEx(Vector2{start_x, start_y},
             Vector2{start_x + 35.0f * std::cos(th_rad), start_y + 35.0f * std::sin(th_rad)}, 3.0f,
             Color{56, 189, 248, 255});
  DrawText("START", static_cast<int>(start_x - 20.0f), static_cast<int>(start_y - 42.0f), 14,
           Color{56, 189, 248, 255});
}

}  // namespace lf::sim
