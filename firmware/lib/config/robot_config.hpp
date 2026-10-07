#pragma once

#include <array>
#include <cmath>

#include "config/constants.hpp"

namespace lf {

struct EnvironmentConfig {
  float obstacle_length_mm = 105.0f;
  float obstacle_width_mm = 60.0f;
  float safety_margin_mm = 25.0f;
  float robot_width_mm = 138.0f;
  float obstacle_merge_dist_mm = 250.0f;

  // Bán kính bao ngoại tiếp vật cản: R_obs = 0.5 * sqrt(105^2 + 60^2) = 60.5 mm
  [[nodiscard]] float obstacle_radius_mm() const {
    return 0.5f * std::sqrt(obstacle_length_mm * obstacle_length_mm +
                            obstacle_width_mm * obstacle_width_mm);
  }

  // Bán kính hành lang an toàn tránh va chạm cho mọi góc xoay theta [0, 360 độ]
  // R_clearance = R_obs + (W_robot / 2) + Margin = 60.5 + 69.0 + 25.0 = 154.5 mm ~ 155 mm
  [[nodiscard]] float clearance_radius_mm() const {
    return obstacle_radius_mm() + (0.5f * robot_width_mm) + safety_margin_mm;
  }
};

struct KinematicsConfig {
  float wheel_diameter_m = 0.034f;
  float wheel_base_m = 0.115f;
  float sensor_lookahead_m = 0.055f;
  float ticks_per_revolution = 560.0f;

  [[nodiscard]] constexpr float meters_per_tick() const {
    return (constants::kPi * wheel_diameter_m) / ticks_per_revolution;
  }
};

struct LineSensorConfig {
  static constexpr int kSensorCount = 8;
  float line_presence_threshold = 0.15f;
  float finish_line_threshold = 0.75f;
  int min_finish_eyes = 7;
  float cross_marker_threshold = 0.45f;
  int cross_marker_min_eyes = 6;
  float branch_cluster_threshold = 0.35f;
  float optical_confidence_k = 10.0f;

  // Trọng số vị trí hình học của 8 cảm biến:
  // Với 8 mắt đối xứng qua tâm x = 0 và bước mắt d = 9.0 mm:
  // w_i = (i - (N - 1) / 2) * d = (i - 3.5f) * 9.0 mm
  // w = {-31.5, -22.5, -13.5, -4.5, +4.5, +13.5, +22.5, +31.5} mm
  std::array<float, kSensorCount> weights_mm = {
    -31.5f, -22.5f, -13.5f, -4.5f, 4.5f, 13.5f, 22.5f, 31.5f,
  };
};

struct PIDConfig {
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  float out_min = -1.0f;
  float out_max = 1.0f;
  float derivative_lpf_alpha = 0.88f;
  float speed_scale_min = 0.35f;
  float speed_scale_max = 1.00f;
  float speed_scale_v_ref = 0.80f;
};

struct ObstacleConfig {
  float front_detect_dist_m = 0.35f;
  float approach_stop_dist_m = 0.07f;
  float bypass_clearance_m = 0.22f;
  float bypass_length_m = 0.82f;
  float bypass_speed_mps = 0.45f;
  float min_valid_tof_m = 0.01f;
  float danger_zone_dist_m = 0.20f;
  float evasion_cooldown_duration_s = 1.0f;
  float recovery_speed_mps = 0.35f;
  float recovery_steer_rad = 0.12f;
  float repulsion_gain = 0.10f;
  float side_tof_repulsion_gain = 0.05f;
};

struct ProfilerConfig {
  float safe_map_speed_mps = 0.80f;
  float max_straight_speed_mps = 2.50f;
  float launch_min_speed_mps = 0.80f;
  float corner_grip_mu = 0.80f;
  float max_accel_mps2 = 3.00f;
  float max_brake_mps2 = 4.50f;
  float gravity_mps2 = 9.81f;
};

struct TrackingConfig {
  float min_finish_arm_dist_m = 1.0f;
  float finish_zone_length_m = 0.125f;
  float finish_debounce_time_s = 0.035f;
  float finish_speed_factor = 0.5f;
  float line_loss_timeout_s = 0.50f;
  float inertial_decay_lambda = 2.0f;
};

struct RobotConfig {
  EnvironmentConfig environment{};
  KinematicsConfig kinematics{};
  LineSensorConfig line_sensor{};
  PIDConfig line_pid{0.040f, 0.0f, 0.004f, -2.5f, 2.5f, 0.80f};
  ObstacleConfig obstacle{};
  ProfilerConfig profiler{};
  TrackingConfig tracking{};
};

}  // namespace lf
