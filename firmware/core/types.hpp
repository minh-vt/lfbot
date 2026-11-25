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

// The LED only ever shows a steady colour: no blinking, nothing to count.
struct Outputs {
  MotorOutput motor;
  Colour led = Colour::kOff;
};

// What the drive is asked to do: how hard to push forward and how hard to turn.
// Both are duty, -1..1. Positive turn is a left turn (right wheel faster).
// The drive applies them directly; there is no inner speed loop and no motor model,
// because that loop's lag is what made the robot react slowly.
struct Target {
  float forward = 0.0F;
  float turn = 0.0F;
};

// What the robot is doing, as the button sees it: one press calibrates, a second
// press races, any press while moving stops.
enum class Phase : std::uint8_t {
  kIdle,
  kSettling,     // standing still while the gyro bias is measured
  kCalibrating,  // swinging over the line to learn each eye's floor and line levels
  kReady,        // calibrated and waiting for the second press
  kRacing,
  kStopped,
  kFinished,
};

// Where the steering aim comes from this tick (ADR-026). Logged, so a run can be
// read back as "followed the line, held through a crossing, searched, ...".
enum class Guide : std::uint8_t {
  kLine,      // the bar sees the line
  kMemory,    // most of the bar is dark: keep the last aim through it
  kSearch,    // the line left through a side: turn that way until it is back
  kStraight,  // the line ended mid-bar: roll on straight to tell a gap from the end
  kVirtual,   // going around a brick on a line drawn in memory
};

// What the bar sees, after calibration.
enum class LineShape : std::uint8_t {
  kNone,
  kLine,
  kSplit,  // two separate dark patches: a crossing or a fork
  kWide,   // dark under most of the bar: a crossing line or a dark area
};

// Why the robot is not moving. Every stop has exactly one of these, and it ends
// up in the log, so "red LED" is never the whole story.
enum class StopReason : std::uint8_t {
  kNone,
  kButton,
  kStall,
  kBattery,
  kGyroStale,
  kGyroDisagrees,
  kNotStill,
  kScanFailed,  // too few eyes could tell the line from the floor
  kIrStale,
  kLineLost,
  kWideTooLong,
  kFinish,
};

}  // namespace lf
