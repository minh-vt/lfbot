#include "robot.hpp"

#include <cmath>

#include "drive.hpp"
#include "race.hpp"

namespace lf {
namespace {

RaceGeometry make_race_geometry(const Calibration& calibration) noexcept {
  RaceGeometry geometry;
  if (!validate_calibration(calibration)) {
    return geometry;
  }
  const float scale = 1.0F / calibration.wheelbase_mm;
  for (std::size_t i = 0; i < kIrSensorCount; ++i) {
    geometry.ir_position[i] = {
      calibration.ir_position_mm[i].x * scale,
      calibration.ir_position_mm[i].y * scale,
    };
  }
  geometry.obstacle_radius = calibration.obstacle_radius_mm * scale;
  geometry.body_radius = std::hypot(calibration.half_length_mm, calibration.half_width_mm) * scale;
  geometry.tof_forward = calibration.tof_forward_mm * scale;
  geometry.tof_half_fov = calibration.tof_half_fov_rad;
  geometry.wheel_resolution = calibration.wheel_mm_per_tick * scale;
  geometry.ir_spot_radius = calibration.ir_spot_radius_mm * scale;
  geometry.line_half_width = 0.5F * calibration.line_width_mm * scale;
  geometry.tof_error = calibration.tof_error_mm * scale;
  const double unit = static_cast<double>(calibration.reference_speed_mm_s) *
                      static_cast<double>(calibration.reference_speed_mm_s);
  const auto normalize_acceleration = [&](float mm_s2) {
    return static_cast<float>(static_cast<double>(mm_s2) * calibration.wheelbase_mm / unit);
  };
  geometry.drive_acceleration = normalize_acceleration(calibration.drive_acceleration_mm_s2);
  geometry.brake_deceleration = normalize_acceleration(calibration.brake_deceleration_mm_s2);
  // The estimator bounds any wheel acceleration, so a brake event counts too.
  geometry.wheel_acceleration = std::max(geometry.drive_acceleration, geometry.brake_deceleration);
  geometry.motor_response =
    static_cast<float>(static_cast<double>(calibration.motor_response_s) *
                       calibration.reference_speed_mm_s / calibration.wheelbase_mm);
  geometry.duty_scale = calibration.duty_speed_mm_s / calibration.reference_speed_mm_s;
  geometry.ir_interval =
    static_cast<float>(static_cast<double>(calibration.ir_period_s) *
                       calibration.reference_speed_mm_s / calibration.wheelbase_mm);
  geometry.lookahead = 0.0F;
  geometry.sensor_span_sum = 0.0F;
  geometry.sensor_span_half = 0.0F;
  for (const auto& position : geometry.ir_position) {
    geometry.lookahead = std::max(geometry.lookahead, position.x);
    geometry.sensor_span_sum += std::abs(position.y);
    geometry.sensor_span_half = std::max(geometry.sensor_span_half, std::abs(position.y));
  }
  return geometry;
}

}  // namespace

struct Robot::State {
  State(const Calibration& input, const RaceTuning& tuning) noexcept
    : drive(input),
      race(make_race_geometry(input), tuning),
      tuning_valid(validate_race_tuning(tuning)) {
    observation.status = race.status();
    if (!tuning_valid) {
      observation.status.phase = RacePhase::kFault;
      observation.status.reason = StopReason::kInvalidConfiguration;
    }
    if (!drive.calibration_valid()) {
      observation.status.phase = RacePhase::kFault;
      observation.status.reason = StopReason::kInvalidCalibration;
    }
  }

  Drive drive;
  Race race;
  RobotObservation observation;
  bool tuning_valid;
};

MotorOutput Robot::drive(const SensorMeasurements& measurements, RaceRequest request) noexcept {
  MotorOutput disabled;
  disabled.deadline_us = measurements.time_us;
  state_.observation.time_us = measurements.time_us;
  if (!state_.tuning_valid) {
    state_.observation.status.phase = RacePhase::kFault;
    state_.observation.status.reason = StopReason::kInvalidConfiguration;
    return disabled;
  }
  if (!state_.drive.calibration_valid()) {
    state_.observation.status.phase = RacePhase::kFault;
    state_.observation.status.reason = StopReason::kInvalidCalibration;
    return disabled;
  }
  if (request == RaceRequest::kReset) {
    state_.drive.reset_session();
  }
  SensorReadings readings;
  StopReason fault = StopReason::kNone;
  if (!state_.drive.read_sensors(measurements, readings, fault)) {
    state_.observation.status.phase = RacePhase::kFault;
    state_.observation.status.reason = fault;
    return disabled;
  }
  const MotorDemand command = state_.race.drive(readings, request);
  state_.observation.status = state_.race.status();
  state_.observation.ir = readings.ir;
  state_.observation.tof = readings.tof;
  const MotorOutput output = state_.drive.make_output(
    command, measurements.time_us, state_.observation.status.phase == RacePhase::kRunning, fault);
  if (fault != StopReason::kNone) {
    state_.observation.status.phase = RacePhase::kFault;
    state_.observation.status.reason = fault;
  }
  return output;
}

void Robot::prepare_run() noexcept {
  state_.drive.prepare_run();
  state_.observation.status = state_.race.status();
}

void Robot::finish_run() noexcept {
  state_.race.finish_run();
  state_.observation.status = state_.race.status();
}

const RobotObservation& Robot::observe() const noexcept {
  return state_.observation;
}

void run_robot(const Calibration& calibration, RobotSession& session,
               const RaceTuning& tuning) noexcept {
  // Native software limit, not a measured ESP32 stack allowance.
  static_assert(sizeof(Robot::State) <= 128UL * 1024UL, "robot state exceeds native memory limit");
  session.disable_motors();
  Robot::State state(calibration, tuning);
  Robot robot(state);
  session.run(robot);
  session.disable_motors();
}

}  // namespace lf
