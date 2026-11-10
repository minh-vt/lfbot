#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace lf {

inline constexpr std::size_t kIrSensorCount = 8;
inline constexpr std::size_t kTofSensorCount = 1;

// A ToF no-return carries its configured limit and acquisition identity, not
// an observed surface or a guarantee that its whole field of view is free.
enum class ReadingStatus : std::uint8_t { kValid, kMissing, kOutOfRange, kInvalid, kNoReturn };
enum class MotorMode : std::uint8_t { kCoast, kDrive, kBrake };
enum class RaceRequest : std::uint8_t { kNone, kStart, kAbort, kReset };
enum class RacePhase : std::uint8_t { kIdle, kRunning, kDone, kFault };
enum class StopReason : std::uint8_t {
  kNone,
  kNotReady,
  kInvalidCalibration,
  kInvalidTime,
  kInvalidOutput,
  kSensorFault,
  kAborted,
  kBlocked,
  kNoFeasiblePath,  // diagnosed navigation stop: no checked motion is available
  kInvalidConfiguration,
};

struct RaceStatus {
  RacePhase phase = RacePhase::kIdle;
  StopReason reason = StopReason::kNotReady;
  std::uint8_t runs_used = 0;
};

struct Point2 {
  float x = 0.0F;
  float y = 0.0F;
};

// Dimensionful acquisition; timestamps use the packet's monotonic microsecond clock.
// Reusing a sample must preserve its timestamp and sequence, including counter wrap.
template <typename T>
struct SensorSample {
  T value{};
  std::uint64_t time_us = 0;
  std::uint32_t sequence = 0;
  ReadingStatus status = ReadingStatus::kMissing;
};

struct SensorMeasurements {
  std::uint64_t time_us = 0;
  std::array<SensorSample<float>, kIrSensorCount> ir;
  std::array<SensorSample<float>, kTofSensorCount> tof;
  std::array<SensorSample<std::uint32_t>, 2> wheel_counters;
};

// Immutable session scales in mm and mm/s. Defaults deliberately reject actuation.
// Sensor placement and the brick envelope are calibration/domain inputs; the
// values shipped with the simulator are proposals, not measured hardware data.
struct Calibration {
  float wheelbase_mm = 0.0F;
  float half_length_mm = 0.0F;
  float half_width_mm = 0.0F;
  float reference_speed_mm_s = 0.0F;
  float wheel_mm_per_tick = 0.0F;
  float ir_spot_radius_mm = 0.0F;
  std::array<float, kIrSensorCount> ir_white{};
  std::array<float, kIrSensorCount> ir_black{};
  std::array<Point2, kIrSensorCount> ir_position_mm{};  // x forward, y left
  float tof_forward_mm = 0.0F;
  float tof_range_mm = 0.0F;
  float tof_half_fov_rad = 0.0F;
  float obstacle_radius_mm = 0.0F;  // circumscribed brick envelope, domain input
  float line_width_mm = 20.0F;      // manual §4: 2 cm, not inferred from the asset
  float tof_error_mm = 0.0F;        // robot-known bound; fixture sets it explicitly
  // Nominal drive model, robot-known from the motor/gearing spec. The plant has
  // its own parameters; these are what the core is allowed to assume.
  float duty_speed_mm_s = 0.0F;           // steady per-wheel speed at duty 1
  float motor_response_s = 0.0F;          // first-order wheel-speed response
  float drive_acceleration_mm_s2 = 0.0F;  // commandable accelerate/decelerate limit
  float brake_deceleration_mm_s2 = 0.0F;  // active-brake deceleration limit
  float ir_period_s = 0.0F;               // acquisition cadence, independent of delivery
};

// Track-dependent choices in L0/V0/T0 units; not measured hardware limits.
// Single source for calibration checks, shared by Drive (core) and sim loaders
// so one set of rules cannot drift between platforms.
[[nodiscard]] inline bool validate_calibration(const Calibration& calibration) noexcept {
  if (!std::isfinite(calibration.duty_speed_mm_s) || calibration.duty_speed_mm_s <= 0.0F ||
      !std::isfinite(calibration.motor_response_s) || calibration.motor_response_s <= 0.0F ||
      !std::isfinite(calibration.drive_acceleration_mm_s2) ||
      calibration.drive_acceleration_mm_s2 <= 0.0F ||
      !std::isfinite(calibration.brake_deceleration_mm_s2) ||
      calibration.brake_deceleration_mm_s2 <= 0.0F || !std::isfinite(calibration.ir_period_s) ||
      calibration.ir_period_s <= 0.0F || !std::isfinite(calibration.line_width_mm) ||
      calibration.line_width_mm <= 0.0F || !std::isfinite(calibration.tof_error_mm) ||
      calibration.tof_error_mm < 0.0F || !std::isfinite(calibration.ir_spot_radius_mm) ||
      calibration.ir_spot_radius_mm <= 0.0F || !std::isfinite(calibration.half_length_mm) ||
      calibration.half_length_mm <= 0.0F || !std::isfinite(calibration.half_width_mm) ||
      calibration.half_width_mm <= 0.0F || !std::isfinite(calibration.wheelbase_mm) ||
      calibration.wheelbase_mm <= 0.0F || !std::isfinite(calibration.reference_speed_mm_s) ||
      calibration.reference_speed_mm_s <= 0.0F || !std::isfinite(calibration.wheel_mm_per_tick) ||
      calibration.wheel_mm_per_tick <= 0.0F || !std::isfinite(calibration.tof_forward_mm) ||
      calibration.tof_forward_mm <= 0.0F || !std::isfinite(calibration.tof_range_mm) ||
      calibration.tof_range_mm <= 0.0F || !std::isfinite(calibration.tof_half_fov_rad) ||
      calibration.tof_half_fov_rad <= 0.0F || calibration.tof_half_fov_rad >= 1.57079633F ||
      !std::isfinite(calibration.obstacle_radius_mm) || calibration.obstacle_radius_mm <= 0.0F) {
    return false;
  }
  const auto scale = static_cast<double>(calibration.wheelbase_mm);
  const auto unit = static_cast<double>(calibration.reference_speed_mm_s) *
                    static_cast<double>(calibration.reference_speed_mm_s);
  const auto acceleration =
    static_cast<float>(static_cast<double>(calibration.drive_acceleration_mm_s2) * scale / unit);
  const auto deceleration =
    static_cast<float>(static_cast<double>(calibration.brake_deceleration_mm_s2) * scale / unit);
  const auto response = static_cast<float>(static_cast<double>(calibration.motor_response_s) *
                                           calibration.reference_speed_mm_s / scale);
  const auto duty_scale =
    static_cast<float>(calibration.duty_speed_mm_s / calibration.reference_speed_mm_s);
  const auto ir_period =
    static_cast<float>(static_cast<double>(calibration.ir_period_s) *
                       calibration.reference_speed_mm_s / calibration.wheelbase_mm);
  if (!std::isfinite(acceleration) || acceleration <= 0.0F || !std::isfinite(deceleration) ||
      deceleration <= 0.0F || !std::isfinite(response) || response <= 0.0F ||
      !std::isfinite(duty_scale) || duty_scale <= 0.0F || !std::isfinite(ir_period) ||
      ir_period <= 0.0F || !std::isfinite(calibration.line_width_mm / calibration.wheelbase_mm) ||
      !std::isfinite(calibration.tof_error_mm / calibration.wheelbase_mm) ||
      !std::isfinite(calibration.ir_spot_radius_mm / calibration.wheelbase_mm) ||
      !std::isfinite(calibration.wheel_mm_per_tick / calibration.wheelbase_mm) ||
      !std::isfinite(calibration.tof_forward_mm / calibration.wheelbase_mm) ||
      !std::isfinite(calibration.obstacle_radius_mm / calibration.wheelbase_mm) ||
      !std::isfinite(std::hypot(calibration.half_length_mm, calibration.half_width_mm) /
                     calibration.wheelbase_mm) ||
      !std::isfinite(calibration.wheelbase_mm / calibration.reference_speed_mm_s) ||
      calibration.wheelbase_mm / calibration.reference_speed_mm_s <= 0.0F ||
      !std::isfinite(calibration.tof_range_mm / calibration.wheelbase_mm) ||
      static_cast<double>(calibration.reference_speed_mm_s) * 180.0 /
          calibration.wheel_mm_per_tick >
        std::numeric_limits<std::int32_t>::max()) {
    return false;
  }
  for (std::size_t i = 0; i < kIrSensorCount; ++i) {
    if (!std::isfinite(calibration.ir_white[i]) || !std::isfinite(calibration.ir_black[i]) ||
        calibration.ir_black[i] == calibration.ir_white[i] ||
        !std::isfinite(calibration.ir_position_mm[i].x) ||
        !std::isfinite(calibration.ir_position_mm[i].y) ||
        !std::isfinite(calibration.ir_position_mm[i].x / calibration.wheelbase_mm) ||
        !std::isfinite(calibration.ir_position_mm[i].y / calibration.wheelbase_mm)) {
      return false;
    }
  }
  return true;
}

// Measure-once robot physics. Control gains are derived from these in Race,
// never taken as hand-tuned kp/kd. Values are unmeasured placeholders until
// hardware calibration exists.
struct RobotTuning {
  // Slip boundary: max speed fraction at a path curvature |κ| (1/L0), from a
  // circle test. Stub values, not hardware data.
  std::array<float, 4> slip_curvature = {0.0F, 0.8F, 1.6F, 2.4F};
  std::array<float, 4> slip_limit = {1.0F, 0.7F, 0.45F, 0.3F};
  // Free-rolling error budget from the 0.85-traction fixture, not a guarantee
  // for contact/wheel-spin. Hardware evidence is still deferred.
  float wheel_travel_error = 0.15F;
  float ir_noise_bound = 0.02F;  // full-scale bound from the declared sensor stress fixture
};

// Per-track trim, in L0/V0/T0 units.
struct TrackTuning {
  float clearance_margin = 0.15F;  // extra gap / L0 beyond brick+body radius
};

struct RaceTuning {
  RobotTuning robot;
  TrackTuning track;
};

// Shared by the core and any caller that checks a plan: a pure data check, no
// hardware or controller dependency.
[[nodiscard]] inline bool validate_race_tuning(const RaceTuning& tuning) noexcept {
  for (std::size_t i = 0; i < tuning.robot.slip_curvature.size(); ++i) {
    if (!std::isfinite(tuning.robot.slip_curvature[i]) ||
        !std::isfinite(tuning.robot.slip_limit[i]) || tuning.robot.slip_limit[i] < 0.0F ||
        tuning.robot.slip_limit[i] > 1.0F) {
      return false;
    }
    if (i > 0 && tuning.robot.slip_curvature[i] <= tuning.robot.slip_curvature[i - 1]) {
      return false;
    }
  }
  return std::isfinite(tuning.robot.ir_noise_bound) && tuning.robot.ir_noise_bound >= 0.0F &&
         tuning.robot.ir_noise_bound < 1.0F && std::isfinite(tuning.robot.wheel_travel_error) &&
         tuning.robot.wheel_travel_error >= 0.0F && tuning.robot.wheel_travel_error <= 1.0F &&
         std::isfinite(tuning.track.clearance_margin) && tuning.track.clearance_margin >= 0.0F;
}

struct MotorOutput {
  float left_duty = 0.0F;
  float right_duty = 0.0F;
  std::uint64_t deadline_us = 0;
  MotorMode mode = MotorMode::kCoast;
};

struct SensorObservation {
  float value = 0.0F;
  float age = 0.0F;  // Dimensionless age: seconds / (L0 / V0).
  std::uint32_t sequence = 0;
  ReadingStatus status = ReadingStatus::kMissing;
};

// Control inputs use L0 = wheelbase, T0 = L0/reference_speed; IR is 0 white, 1 black.
// Unlike physical SensorMeasurements, sample ages and elapsed time are in T0.
struct SensorReadings {
  float elapsed_time = 0.0F;
  std::array<SensorObservation, kIrSensorCount> ir;
  std::array<SensorObservation, kTofSensorCount> tof;
  std::array<SensorObservation, 2> wheel_positions;
};

// Before Drive stamps a physical deadline. valid_for_time is a duration in T0.
struct MotorDemand {
  float left_duty = 0.0F;
  float right_duty = 0.0F;
  float valid_for_time = 0.0F;
  MotorMode mode = MotorMode::kCoast;
};

struct RobotObservation {
  std::uint64_t time_us = 0;
  RaceStatus status;
  std::array<SensorObservation, kIrSensorCount> ir;
  std::array<SensorObservation, kTofSensorCount> tof;
};

}  // namespace lf
