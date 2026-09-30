#include "world.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "config/robot_config.hpp"

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
  // Vanh đai an toan ngoai tiep R_clearance tranh va cham cho moi goc xoay
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
  DrawRectangle(0, 0, static_cast<int>(kTrackWidthMm), static_cast<int>(kTrackHeightMm),
                Color{248, 249, 250, 255});
  for (int gx = 0; gx <= static_cast<int>(kTrackWidthMm); gx += 200) {
    DrawLine(gx, 0, gx, static_cast<int>(kTrackHeightMm), Color{225, 230, 238, 255});
  }
  for (int gy = 0; gy <= static_cast<int>(kTrackHeightMm); gy += 200) {
    DrawLine(0, gy, static_cast<int>(kTrackWidthMm), gy, Color{225, 230, 238, 255});
  }

  DrawTexture(track_texture, 0, 0, WHITE);
  DrawRectangleLinesEx(Rectangle{0, 0, kTrackWidthMm, kTrackHeightMm}, 4.0f,
                       Color{71, 85, 105, 255});
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
                        const std::array<float, lf::LineSensorConfig::kSensorCount>& weights_mm) {
  const float cos_th = std::cos(robot_theta);
  const float sin_th = std::sin(robot_theta);

  // Khung xe
  draw_rotated_box(Vector2{robot_x, robot_y}, kRobotLengthMm, kRobotWidthMm, robot_theta * RAD2DEG,
                   Color{26, 32, 44, 230}, Color{56, 189, 248, 255}, 2.0f);

  // Mui ten chi huong mui xe
  const Vector2 fwd_tip = {robot_x + (kRobotLengthMm * 0.46f) * cos_th,
                           robot_y + (kRobotLengthMm * 0.46f) * sin_th};
  const Vector2 fwd_l = {robot_x + (kRobotLengthMm * 0.18f) * cos_th - 16.0f * sin_th,
                         robot_y + (kRobotLengthMm * 0.18f) * sin_th + 16.0f * cos_th};
  const Vector2 fwd_r = {robot_x + (kRobotLengthMm * 0.18f) * cos_th + 16.0f * sin_th,
                         robot_y + (kRobotLengthMm * 0.18f) * sin_th - 16.0f * cos_th};
  DrawTriangle(fwd_tip, fwd_l, fwd_r, Color{56, 189, 248, 220});

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
                    float clearance_radius_mm) {
  for (size_t i = 0; i < obstacles.size(); ++i) {
    const bool is_hovered = (static_cast<int>(i) == hovered_idx);
    draw_hazard_obstacle(obstacles.at(i), is_hovered, clearance_radius_mm);
  }
}

void draw_tof_beams(float bar_center_x, float bar_center_y, Vector2 l_origin, Vector2 r_origin,
                    float robot_theta, const TofHitPoints& hits, const lf::SensorData& in) {
  draw_rotated_box(Vector2{bar_center_x, bar_center_y}, 14.0f, 10.0f, robot_theta * RAD2DEG,
                   Color{15, 23, 42, 255}, Color{56, 189, 248, 255}, 1.0f);
  draw_rotated_box(l_origin, 10.0f, 12.0f, (robot_theta - 1.5707963f) * RAD2DEG,
                   Color{15, 23, 42, 255}, Color{251, 191, 36, 255}, 1.0f);
  draw_rotated_box(r_origin, 10.0f, 12.0f, (robot_theta + 1.5707963f) * RAD2DEG,
                   Color{15, 23, 42, 255}, Color{192, 132, 252, 255}, 1.0f);

  const Color f_col = (in.tof_front_m < 0.15f) ? Color{248, 113, 113, 255}
                                               : ColorAlpha(Color{56, 189, 248, 255}, 0.75f);
  DrawLineEx(Vector2{bar_center_x, bar_center_y}, hits.front,
             (in.tof_front_m < 0.15f) ? 3.0f : 1.8f, f_col);
  DrawCircle(static_cast<int>(hits.front.x), static_cast<int>(hits.front.y),
             (in.tof_front_m < 0.15f) ? 5.5f : 3.5f, f_col);
  if (in.tof_front_m < 0.15f) {
    DrawCircleLines(static_cast<int>(hits.front.x), static_cast<int>(hits.front.y), 9.0f,
                    Color{248, 113, 113, 200});
  }

  const Color l_col = ColorAlpha(Color{251, 191, 36, 255}, 0.85f);
  DrawLineEx(l_origin, hits.left, 1.8f, l_col);
  DrawCircle(static_cast<int>(hits.left.x), static_cast<int>(hits.left.y), 3.5f, l_col);

  const Color r_col = ColorAlpha(Color{192, 132, 252, 255}, 0.85f);
  DrawLineEx(r_origin, hits.right, 1.8f, r_col);
  DrawCircle(static_cast<int>(hits.right.x), static_cast<int>(hits.right.y), 3.5f, r_col);
}

}  // namespace lf::sim
