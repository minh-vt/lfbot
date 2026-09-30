#pragma once

#include <array>
#include <cstdint>

namespace lf {

enum class RobotMode : uint8_t {
  kIdle = 0,
  kCalibrating,
  kAutoTuning,
  kRun1Map,
  kRun2Sprint,
  kEmergencyStop
};

enum class SegmentType : uint8_t {
  kStraight = 0,
  kCurveLeft,
  kCurveRight,
  kObstacleZone,
  kFinishZone
};

struct SensorData {
  static constexpr int kMaxSensors = 8;
  std::array<float, kMaxSensors> ir_raw{};
  float tof_front_m = 0.0f;
  float tof_left_m = 0.0f;
  float tof_right_m = 0.0f;
  float tof_side_m = 0.0f;  // Active bypass side alias (defaults to tof_right_m)
  int32_t enc_left_ticks = 0;
  int32_t enc_right_ticks = 0;
  float dt = 0.0f;
};

struct MotorCmd {
  float left_pwm;
  float right_pwm;
  bool active_brake;
};

struct MapSegment {
  SegmentType type;
  float start_dist_m;
  float length_m;
  float curvature_rad_m;
  float v_max_allowed;
  float v_entry_target;
  float v_exit_target;
};

struct WheelTelemetry {
  float target_speed_mps = 0.0f;
  float actual_speed_mps = 0.0f;
  int32_t enc_ticks = 0;
  float pwm_duty = 0.0f;
};

struct ObstacleTelemetry {
  float tof_front_m = 0.0f;
  float tof_left_m = 0.0f;
  float tof_right_m = 0.0f;
  float nearest_dist_m = 0.0f;
  float orientation_deg = 0.0f;
  bool detected = false;
};

struct TrajectoryPoint {
  float x_mm = 0.0f;
  float y_mm = 0.0f;
  float target_speed_mps = 0.0f;
  float curvature_rad_m = 0.0f;
};

struct Telemetry {
  RobotMode mode = RobotMode::kIdle;
  float centroid_pos_mm = 0.0f;
  float battery_v = 0.0f;
  float current_velocity_mps = 0.0f;
  float total_distance_m = 0.0f;
  int current_segment_idx = 0;
  bool line_lost = false;
  uint8_t fsm_state = 0;
  float heading_rad = 0.0f;
  float angular_velocity_radps = 0.0f;
  WheelTelemetry wheel_l{};
  WheelTelemetry wheel_r{};
  ObstacleTelemetry obstacle{};
};

}  // namespace lf
