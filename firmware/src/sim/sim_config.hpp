#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace lf::sim {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kWheelSeparationMm = 138.0f;
constexpr float kMaxWheelSpeedMps = 1.0f;    // duty=1 → 1.0 m/s
constexpr float kSensorLookaheadMm = 55.0f;  // cảm biến cách trục sau
constexpr float kTicksPerMeter = 50.0f;      // 1 tick = 20 mm (chỉ số plan ≤ kMaxPlanTicks)
constexpr float kRobotHalfLengthMm = 72.5f;  // robot 145mm
constexpr float kRobotHalfWidthMm = 69.0f;   // robot 138mm
// Bán kính bao robot = nửa đường chéo = sqrt(72.5² + 69²) ≈ 100.1mm.
constexpr float kRobotRadiusMm = 100.1f;
constexpr float kLineDarkThreshold = 128.0f;  // < 128 = vạch (đen)
constexpr float kLineLostTimeoutS = 5.0f;
constexpr float kTimeLimitS = 180.0f;
constexpr float kFinishStopSpeedMps = 0.05f;

struct ObstacleSpec {
  float x_mm = 0.0f;
  float y_mm = 0.0f;
  float width_mm = 105.0f;
  float height_mm = 60.0f;
  float angle_deg = 0.0f;
};

struct TrackManifest {
  std::string name;
  std::string image_path;
  float arena_width_mm = 6000.0f;
  float arena_height_mm = 1500.0f;
  float start_x_mm = 0.0f;
  float start_y_mm = 0.0f;
  float start_theta_deg = 0.0f;
  float finish_min_x = 0.0f;
  float finish_max_x = 0.0f;
  float finish_min_y = 0.0f;
  float finish_max_y = 0.0f;
  std::vector<ObstacleSpec> obstacles;
};

}
