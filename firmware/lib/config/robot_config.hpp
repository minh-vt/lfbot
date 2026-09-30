#pragma once

#include <array>
#include <cmath>

namespace lf {

struct EnvironmentConfig {
  float obstacle_length_mm = 105.0f;
  float obstacle_width_mm = 60.0f;
  float line_width_mm = 20.0f;
  float arena_width_mm = 2400.0f;
  float arena_height_mm = 1600.0f;
  float safety_margin_mm = 25.0f;
  float robot_width_mm = 138.0f;

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
  float sensor_lookahead_m = 0.080f;
  float ticks_per_revolution = 560.0f;

  [[nodiscard]] constexpr float meters_per_tick() const {
    return (3.14159265f * wheel_diameter_m) / ticks_per_revolution;
  }
};

struct LineSensorConfig {
  static constexpr int kSensorCount = 8;
  float sensor_pitch_mm = 9.0f;
  float line_presence_threshold = 0.15f;
  float finish_line_threshold = 0.75f;
  int min_finish_eyes = 7;

  // Trọng số vị trí hình học của 8 cảm biến:
  // Với 8 mắt đối xứng qua tâm x = 0 và bước mắt d = 9.0 mm:
  // w_i = (i - (N - 1) / 2) * d = (i - 3.5f) * 9.0 mm
  // w = {-31.5, -22.5, -13.5, -4.5, +4.5, +13.5, +22.5, +31.5} mm
  std::array<float, kSensorCount> weights_mm = {-31.5f, -22.5f, -13.5f, -4.5f,
                                                4.5f,   13.5f,  22.5f,  31.5f};
};

struct PIDConfig {
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  float out_min = -1.0f;
  float out_max = 1.0f;
  float derivative_lpf_alpha = 0.7f;
};

struct ObstacleConfig {
  float front_detect_dist_m = 0.28f;
  float approach_stop_dist_m = 0.07f;
  float bypass_clearance_m = 0.18f;
  float bypass_length_m = 0.75f;
  float bypass_speed_mps = 0.55f;
  float emergency_timeout_s = 4.20f;
  float emergency_spin_pwm = 0.30f;
  float min_valid_tof_m = 0.01f;
  float wheel_base_m = 0.138f;
  float max_straight_speed_mps = 2.50f;
};

struct ProfilerConfig {
  static constexpr int kMaxSegments = 64;
  float safe_map_speed_mps = 0.80f;
  float max_straight_speed_mps = 2.50f;
  float obstacle_zone_speed_mps = 1.00f;
  float launch_min_speed_mps = 0.80f;
  float corner_grip_mu = 0.80f;
  float max_accel_mps2 = 3.00f;
  float max_brake_mps2 = 4.50f;
  float lookahead_distance_m = 0.12f;
  float gravity_mps2 = 9.81f;
  float fallback_curve_radius_m = 10.0f;
};

struct TrackingConfig {
  float curve_centroid_threshold_mm = 12.0f;
  float max_centroid_curvature_mm = 35.0f;
  float max_curvature_rad_m = 10.0f;
  float min_segment_length_m = 0.15f;
  float min_finish_arm_dist_m = 1.0f;
  float finish_zone_length_m = 0.155f;
  float default_loop_dt_s = 0.002f;
  float min_valid_dt_s = 0.0001f;
  float max_error_mm = 32.0f;
  float min_reactive_speed_scale = 0.40f;
};

struct RobotConfig {
  EnvironmentConfig environment{};
  KinematicsConfig kinematics{};
  LineSensorConfig line_sensor{};
  PIDConfig line_pid{0.045f, 0.0005f, 0.008f, -1.5f, 1.5f, 0.7f};
  PIDConfig speed_pid_l{0.8f, 0.1f, 0.0f, -1.0f, 1.0f, 0.7f};
  PIDConfig speed_pid_r{0.8f, 0.1f, 0.0f, -1.0f, 1.0f, 0.7f};
  ObstacleConfig obstacle{};
  ProfilerConfig profiler{};
  TrackingConfig tracking{};
};

}  // namespace lf
