#pragma once

#include <cstddef>

#include "config/types.hpp"
#include "raylib.h"
#include "system/robot_coordinator.hpp"
#include "types.hpp"

namespace lf::sim {

void draw_text_crisp(const Font& font, const char* text, float x, float y, float font_size,
                     Color color);

void draw_hud_card(float x, float y, float w, float h, float roundness = 0.2f,
                   Color fill = Color{26, 33, 47, 255}, Color border = Color{45, 56, 78, 255});

void draw_sidebar_background(float sb_x, float sb_w, int screen_h, float ui_scale);

void draw_header_card(float sb_x, float pad_x, float card_w, float ui_scale, float sim_time,
                      bool is_paused, lf::RobotMode mode, bool is_telemetry_mode,
                      uint32_t packets_received, const Font& font_bold, const Font& font_mono);

void draw_telemetry_card(float sb_x, float pad_x, float half_w, float ui_scale, float current_v,
                         float max_speed, const lf::Telemetry& tel, const Font& font_bold);

void draw_centroid_card(float sb_x, float pad_x, float card_w, float ui_scale,
                        const lf::Telemetry& tel, const Font& font_bold);

void draw_sensors_card(float sb_x, float pad_x, float card_w, float ui_scale,
                       const lf::SensorData& in, const Font& font_bold);

void draw_motor_pwm_card(float sb_x, float pad_x, float card_w, float ui_scale, float left_pwm,
                         float right_pwm, const lf::Telemetry& tel, const Font& font_bold,
                         const Font& font_mono);

void draw_tof_card(float sb_x, float pad_x, float card_w, float ui_scale, const lf::SensorData& in,
                   size_t obstacle_count, float hovered_angle_deg, const Font& font_bold,
                   const Font& font_mono);

void draw_lap_comparison_card(float sb_x, float pad_x, float card_w, float ui_scale,
                              const SimRobotState& state, const Font& font_bold,
                              const Font& font_mono);

void draw_tactical_card(float sb_x, float pad_x, float card_w, float ui_scale,
                        const lf::RobotCoordinator& robot, const Font& font_bold,
                        const Font& font_mono);

void draw_toast_notification(float sb_x, float pad_x, float card_w, float ui_scale,
                             const char* toast_msg, const Font& font_bold);

void draw_controls_card(float sb_x, float pad_x, float card_w, float view_h, float ui_scale,
                        bool follow_robot, const Font& font_bold);

}  // namespace lf::sim
