#pragma once

#include <array>
#include <cstdint>

namespace lf {

enum class RobotMode : uint8_t { kIdle = 0, kAutoTune, kRun1Map, kRun2Sprint };

struct Pose2D {
  float x_m = 0.0f;
  float y_m = 0.0f;
  float heading_rad = 0.0f;
};

struct SensorData {
  static constexpr int kMaxSensors = 8;
  std::array<float, kMaxSensors> ir_raw{};
  float tof_front_m = 0.0f;
  float tof_left_m = 0.0f;
  float tof_right_m = 0.0f;
  int32_t enc_left_ticks = 0;
  int32_t enc_right_ticks = 0;
  float dt = 0.0f;
};

struct MotorCmd {
  float left_pwm;
  float right_pwm;
  bool active_brake;
};

struct WheelTelemetry {
  float actual_speed_mps = 0.0f;
  int32_t enc_ticks = 0;
  float pwm_duty = 0.0f;
};

struct ObstacleTelemetry {
  float tof_front_m = 0.0f;
  float tof_left_m = 0.0f;
  float tof_right_m = 0.0f;
  float nearest_dist_m = 0.0f;
  bool detected = false;
  bool collision = false;
};

struct ObstacleGeometry {
  float x_mm = 0.0f;
  float y_mm = 0.0f;
  float width_mm = 105.0f;
  float height_mm = 60.0f;
  float angle_deg = 0.0f;
};

struct TrajectoryPoint {
  float x_mm = 0.0f;
  float y_mm = 0.0f;
  float target_speed_mps = 0.0f;
  float curvature_rad_m = 0.0f;
  float s_m = 0.0f;
  float heading_rad = 0.0f;
};

struct Telemetry {
  RobotMode mode = RobotMode::kIdle;
  float centroid_pos_mm = 0.0f;
  float battery_v = 0.0f;
  float current_velocity_mps = 0.0f;
  float total_distance_m = 0.0f;
  int current_segment_idx = 0;
  bool line_lost = false;
  bool collision = false;
  uint8_t bypass_state = 0;
  float heading_rad = 0.0f;
  float angular_velocity_radps = 0.0f;
  float cross_track_error_m = 0.0f;
  float heading_error_rad = 0.0f;
  float target_velocity_mps = 0.0f;
  float stanley_steer_rad = 0.0f;
  WheelTelemetry wheel_l{};
  WheelTelemetry wheel_r{};
  ObstacleTelemetry obstacle{};
  int tuner_cycles = 0;
  float tuner_amplitude_mm = 0.0f;
};

}  // namespace lf
