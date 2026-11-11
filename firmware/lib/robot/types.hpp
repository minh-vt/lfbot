#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace lf {

constexpr std::size_t kIrCount = 8;
constexpr std::size_t kWheelCount = 2;

enum class SampleStatus : std::uint8_t {
  kUnavailable,
  kValid,
  kInvalid,
  kOverflow,
};

// A measurement plus the interval it was acquired in.
template <typename T>
struct Sample {
  T value{};
  std::uint64_t earliest_us = 0;
  std::uint64_t latest_us = 0;
  std::uint32_t sequence = 0;
  std::uint32_t period_us = 0;
  SampleStatus status = SampleStatus::kUnavailable;
};

// Raw as the drivers deliver it: wiring signs are applied in core, in one place.
struct Measurements {
  Sample<std::array<std::uint16_t, kIrCount>> ir;
  Sample<std::array<std::int32_t, kWheelCount>> encoders;
  Sample<float> gyro_z_rad_s;
  Sample<float> distance_mm;
  Sample<float> battery_v;
  std::uint8_t range_status = 255;
  std::uint32_t adc_overflows = 0;
};

enum class MotorMode : std::uint8_t {
  kCoast,
  kBrake,
  kDrive,
};

// duty is -1..1 exactly as the driver sees it: positive drives IN1. Which way
// that moves the robot is a wiring fact and lives in config.hpp.
struct MotorOutput {
  MotorMode mode = MotorMode::kCoast;
  std::array<float, kWheelCount> duty{};
};

enum class Colour : std::uint8_t {
  kOff,
  kRed,
  kOrange,
  kYellow,
  kGreen,
  kCyan,
  kBlue,
  kPurple,
  kMagenta,
  kWhite,
};

enum class Pattern : std::uint8_t {
  kSteady,
  kBlink,
  kPulse,
};

struct LedPattern {
  Colour colour = Colour::kOff;
  Pattern pattern = Pattern::kSteady;
};

struct Outputs {
  MotorOutput motor;
  LedPattern led;
};

enum class Phase : std::uint8_t {
  kIdle,
  kSettling,  // standing still while the gyro bias is measured
  kRunning,
  kStopped,
};

// Why the robot is not moving. Every stop has exactly one of these, and it ends
// up in the log, so "red LED" is never the whole story.
enum class StopReason : std::uint8_t {
  kNone,
  kDone,
  kButton,
  kStall,
  kBattery,
  kGyroStale,
  kGyroDisagrees,
  kManeuverTimeout,
  kNotStill,
};

}  // namespace lf
