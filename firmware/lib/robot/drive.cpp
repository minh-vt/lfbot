#include "drive.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lf {
namespace {

bool accepts_calibration(const Calibration& calibration) noexcept {
  return validate_calibration(calibration);
}

bool accepts_sample(const SensorSample<float>& input, std::uint64_t now_us) noexcept {
  return input.status == ReadingStatus::kValid && input.time_us <= now_us &&
         std::isfinite(input.value);
}

float measure_sample_age(std::uint64_t now_us, std::uint64_t sample_us,
                         float reference_time) noexcept {
  return static_cast<float>(now_us - sample_us) * 1e-6F / reference_time;
}

SensorObservation normalize_ir(const SensorSample<float>& input, float white, float black,
                               std::uint64_t now_us, float reference_time) noexcept {
  SensorObservation output;
  output.sequence = input.sequence;
  output.status = input.status;
  if (!accepts_sample(input, now_us)) {
    if (input.status == ReadingStatus::kValid) {
      output.status = ReadingStatus::kInvalid;
    }
    return output;
  }
  output.value = (input.value - white) / (black - white);
  output.age = measure_sample_age(now_us, input.time_us, reference_time);
  if (!std::isfinite(output.value) || output.value < -0.05F || output.value > 1.05F) {
    output.status = ReadingStatus::kOutOfRange;
  } else {
    output.value = std::clamp(output.value, 0.0F, 1.0F);
  }
  return output;
}

SensorObservation normalize_tof(const SensorSample<float>& input, float length,
                                std::uint64_t now_us, float reference_time) noexcept {
  SensorObservation output;
  output.sequence = input.sequence;
  output.status = input.status;
  const bool available =
    input.status == ReadingStatus::kValid || input.status == ReadingStatus::kNoReturn;
  if (!available) {
    return output;
  }
  if (input.time_us > now_us || !std::isfinite(input.value) || input.value < 0.0F) {
    output.status = ReadingStatus::kInvalid;
    return output;
  }
  output.value = input.value / length;
  output.age = measure_sample_age(now_us, input.time_us, reference_time);
  return output;
}

SensorObservation normalize_encoder(const SensorSample<std::uint32_t>& input, std::uint64_t now_us,
                                    float reference_time, std::uint32_t& previous, double& position,
                                    bool has_previous, float tick_scale, float length) noexcept {
  SensorObservation output;
  output.sequence = input.sequence;
  output.status = input.status;
  if (input.status != ReadingStatus::kValid || input.time_us > now_us) {
    if (input.status == ReadingStatus::kValid) {
      output.status = ReadingStatus::kInvalid;
    }
    return output;
  }
  if (has_previous) {
    const std::uint32_t wrapped = input.value - previous;
    const std::int64_t delta = wrapped <= 0x7fffffffU
                                 ? static_cast<std::int64_t>(wrapped)
                                 : static_cast<std::int64_t>(wrapped) - 0x100000000LL;
    position += static_cast<double>(delta) * tick_scale / length;
  }
  previous = input.value;
  output.value = static_cast<float>(position);
  output.age = measure_sample_age(now_us, input.time_us, reference_time);
  return output;
}

}  // namespace

Drive::Drive(const Calibration& calibration) noexcept
  : calibration_(calibration), calibration_valid_(accepts_calibration(calibration)) {}

void Drive::reset_session() noexcept {
  has_previous_time_ = false;
  has_counters_.fill(false);
  previous_counters_.fill(0);
  wheel_positions_.fill(0.0F);
}

void Drive::prepare_run() noexcept {
  has_counters_.fill(false);
}

bool Drive::read_sensors(const SensorMeasurements& measurements, SensorReadings& readings,
                         StopReason& fault) noexcept {
  readings = {};
  if (!calibration_valid_) {
    fault = StopReason::kInvalidCalibration;
    return false;
  }
  if (has_previous_time_ && measurements.time_us <= previous_time_us_) {
    fault = StopReason::kInvalidTime;
    return false;
  }
  const float length = calibration_.wheelbase_mm;
  const float reference_time = length / calibration_.reference_speed_mm_s;
  readings.elapsed_time =
    has_previous_time_
      ? static_cast<float>(measurements.time_us - previous_time_us_) * 1e-6F / reference_time
      : 0.0F;
  for (std::size_t i = 0; i < kIrSensorCount; ++i) {
    readings.ir[i] = normalize_ir(measurements.ir[i], calibration_.ir_white[i],
                                  calibration_.ir_black[i], measurements.time_us, reference_time);
  }
  for (std::size_t i = 0; i < kTofSensorCount; ++i) {
    readings.tof[i] =
      normalize_tof(measurements.tof[i], length, measurements.time_us, reference_time);
  }
  for (std::size_t i = 0; i < previous_counters_.size(); ++i) {
    readings.wheel_positions[i] = normalize_encoder(
      measurements.wheel_counters[i], measurements.time_us, reference_time, previous_counters_[i],
      wheel_positions_[i], has_counters_[i], calibration_.wheel_mm_per_tick, length);
    has_counters_[i] =
      has_counters_[i] || readings.wheel_positions[i].status == ReadingStatus::kValid;
  }
  previous_time_us_ = measurements.time_us;
  has_previous_time_ = true;
  return true;
}

MotorOutput Drive::make_output(const MotorDemand& demand, std::uint64_t now_us,
                               bool allowed_to_drive, StopReason& fault) const noexcept {
  fault = StopReason::kNone;
  MotorOutput disabled;
  disabled.deadline_us = now_us;
  if (!calibration_valid_) {
    fault = StopReason::kInvalidCalibration;
    return disabled;
  }
  if (!allowed_to_drive || !std::isfinite(demand.left_duty) || !std::isfinite(demand.right_duty) ||
      !std::isfinite(demand.valid_for_time) || std::abs(demand.left_duty) > 1.0F ||
      std::abs(demand.right_duty) > 1.0F || demand.valid_for_time <= 0.0F) {
    return disabled;
  }
  const double reference_time =
    static_cast<double>(calibration_.wheelbase_mm) / calibration_.reference_speed_mm_s;
  const double validity_us = static_cast<double>(demand.valid_for_time) * reference_time * 1e6;
  if (!std::isfinite(validity_us) || validity_us <= 0.0 ||
      validity_us > static_cast<double>(std::numeric_limits<std::uint64_t>::max() - now_us)) {
    fault = StopReason::kInvalidOutput;
    return disabled;
  }
  return {
    demand.left_duty,
    demand.right_duty,
    now_us + static_cast<std::uint64_t>(validity_us),
    demand.mode,
  };
}

}  // namespace lf
