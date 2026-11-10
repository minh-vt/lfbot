#pragma once

#include <array>
#include <robot_data.hpp>

namespace lf::sim {

// Chassis dimensions retained from the pre-migration UI fixture; curved IR is a
// simulation proposal, not measured hardware calibration. Raster y points down.
struct RobotGeometry {
  float wheelbase_mm = 138.0F;
  float half_length_mm = 72.5F;
  float half_width_mm = 69.0F;
  float tof_forward_mm = 72.5F;
  // Emitter/photo-transistor spot on the floor, not a mathematical point.
  float ir_spot_radius_mm = 4.0F;
  float tof_range_mm = 4000.0F;
  float tof_half_fov_rad = 0.21816616F;
  // Body frame: x forward, y left; convert explicitly to raster y-down.
  std::array<Point2, kIrSensorCount> ir_positions_mm = {
    {
      {53.80479F, -30.80577F},
      {57.57721F, -22.99617F},
      {60.22295F, -14.73756F},
      {61.69026F, -6.18967F},
      {61.69026F, 6.18967F},
      {60.22295F, 14.73756F},
      {57.57721F, 22.99617F},
      {53.80479F, 30.80577F},
    },
  };
};

// Robot-known normalization scale, not instantaneous motor speed. The plant
// has independent response, speed and traction parameters in WorldParameters.
inline constexpr float kSimulatedReferenceSpeedMmS = 500.0F;

inline constexpr RobotGeometry kSimulatedRobot{};

// One mapping for the GUI, headless runs and training. It describes the
// simulated robot only; no measured hardware calibration is claimed.
inline Calibration simulated_calibration() noexcept {
  Calibration calibration;
  calibration.wheelbase_mm = kSimulatedRobot.wheelbase_mm;
  calibration.half_length_mm = kSimulatedRobot.half_length_mm;
  calibration.half_width_mm = kSimulatedRobot.half_width_mm;
  calibration.reference_speed_mm_s = kSimulatedReferenceSpeedMmS;
  calibration.wheel_mm_per_tick = 1.0F;
  calibration.ir_spot_radius_mm = kSimulatedRobot.ir_spot_radius_mm;
  calibration.ir_white.fill(255.0F);
  calibration.ir_black.fill(0.0F);
  calibration.ir_position_mm = kSimulatedRobot.ir_positions_mm;
  calibration.tof_forward_mm = kSimulatedRobot.tof_forward_mm;
  calibration.tof_range_mm = kSimulatedRobot.tof_range_mm;
  calibration.tof_error_mm = 5.0F;  // frozen bounded-noise fixture, not a hardware measurement
  // Frozen nominal drive model, mirrored by WorldParameters at the fixture.
  calibration.duty_speed_mm_s = 500.0F;
  calibration.motor_response_s = 0.060F;
  calibration.drive_acceleration_mm_s2 = 2000.0F;
  calibration.brake_deceleration_mm_s2 = 4000.0F;
  calibration.ir_period_s = 0.002F;
  calibration.tof_half_fov_rad = kSimulatedRobot.tof_half_fov_rad;
  constexpr float kBrickDiagonalMm = 245.10F;  // FAQ brick envelope, domain input
  calibration.obstacle_radius_mm = 0.5F * kBrickDiagonalMm;
  return calibration;
}

}  // namespace lf::sim
