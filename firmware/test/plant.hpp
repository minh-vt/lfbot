#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "config.hpp"
#include "robot.hpp"

// A stand-in robot for closed-loop contract tests: two first-order motors, a
// body that turns by the wheel speed difference, a gyro and encoders reporting
// with the wiring signs of the real robot. It proves the signs and the plumbing,
// not that the real robot will behave.

// A course of straight line segments, in millimetres.
struct Segment {
  float x0;
  float y0;
  float x1;
  float y1;
};

// What the bar reads over floor and over line, per eye. These are test inputs:
// the firmware is given no idea of them.
struct Surface {
  std::array<float, lf::kIrCount> floor{};
  std::array<float, lf::kIrCount> line{};
  static Surface uniform(float floor_level, float line_level) {
    Surface surface;
    for (std::size_t eye = 0; eye < lf::kIrCount; ++eye) {
      // Eyes of one bar differ a little from each other.
      surface.floor[eye] = floor_level + 25.0F * static_cast<float>(eye % 3);
      surface.line[eye] = line_level + 15.0F * static_cast<float>(eye % 2);
    }
    return surface;
  }
};

inline float distance_to_segment(const Segment& s, float x, float y) {
  const float dx = s.x1 - s.x0;
  const float dy = s.y1 - s.y0;
  const float along =
    std::clamp(((x - s.x0) * dx + (y - s.y0) * dy) / (dx * dx + dy * dy), 0.0F, 1.0F);
  return std::hypot(x - (s.x0 + along * dx), y - (s.y0 + along * dy));
}

// The stand-in robot's motor physics, fitted from the real robot on the floor. The
// firmware no longer carries a motor model (it drives duty directly); this is only
// what the simulated plant does when given a duty, so the closed-loop tests have a
// robot to act on.
inline constexpr std::array<float, lf::kWheelCount> kPlantMotorGain = {22480.0F, 21009.0F};
inline constexpr std::array<float, lf::kWheelCount> kPlantMotorTau = {0.62F, 0.55F};
inline constexpr std::array<float, lf::kWheelCount> kPlantMotorDead = {0.21F, 0.22F};

struct Plant {
  static constexpr float kGainScale = 0.85F;
  static constexpr float kExtraDead = 0.05F;
  static constexpr float kGyroBiasRadS = 0.012F;

  std::array<float, lf::kWheelCount> speed{};
  std::array<double, lf::kWheelCount> counts{};
  double yaw_rad = 0.0;
  float gyro_sign = lf::kGyroSign;  // flipped by a test to model a wrong constant
  float battery_v = 11.5F;
  bool jammed = false;
  bool gyro_alive = true;
  std::uint64_t tof_age_us = 0;
  std::uint32_t sequence = 0;

  // Where the robot is on the course, and what the bar would read there. The bar
  // geometry is the firmware's own assembly facts; the line is 20 mm (manual 4.2).
  static constexpr float kEyePitchMm = lf::kEyePitchMm;
  static constexpr float kLineWidthMm = 20.0F;
  static constexpr float kBarAheadMm = lf::kBarAheadMm;
  // The robot's footprint around the axle: 110 mm wide, reaching a little past the bar.
  static constexpr float kHalfWidthMm = 55.0F;
  static constexpr float kRearMm = 30.0F;
  static constexpr float kFrontMm = kBarAheadMm + 10.0F;
  double x_mm = 0.0;
  double y_mm = 0.0;
  std::vector<Segment> course;
  Surface surface = Surface::uniform(3300.0F, 700.0F);
  std::array<bool, lf::kIrCount> saturated{};  // an eye pinned at full scale
  std::uint32_t noise_state = 12345;

  // A brick on the line at y = 0, worst case 205 mm square in plan (FAQ Q-13). The
  // ToF sees its front face; the bar does not see it. brick_hit records any contact.
  double brick_front_x = 0.0;
  bool has_brick = false;
  bool brick_hit = false;
  // Filled squares (a junction node) where the surface reads as line.
  std::vector<std::array<double, 4>> dark_squares;  // {cx, cy, half, unused}

  void advance(const lf::Outputs& outputs, float dt) {
    for (std::size_t wheel = 0; wheel < lf::kWheelCount; ++wheel) {
      // Braking shorts the motor and stops the wheel in the test plant.
      if (outputs.motor.mode == lf::MotorMode::kBrake) {
        speed[wheel] = 0.0F;
      }
      float duty = outputs.motor.mode == lf::MotorMode::kDrive ? outputs.motor.duty[wheel] : 0.0F;
      if (lf::kInvertWheel[wheel]) {
        duty = -duty;
      }
      const float dead = kPlantMotorDead[wheel] + kExtraDead;
      const float magnitude = std::max(0.0F, std::abs(duty) - dead);
      const float steady =
        (duty >= 0.0F ? 1.0F : -1.0F) * magnitude * kPlantMotorGain[wheel] * kGainScale;
      speed[wheel] += (steady - speed[wheel]) * dt / kPlantMotorTau[wheel];
      if (jammed) {
        speed[wheel] = 0.0F;
      }
      counts[wheel] += static_cast<double>(speed[wheel] * dt);
    }
    const float forward_mm_s = 0.5F * (speed[0] + speed[1]) * lf::kMmPerCount;
    x_mm += static_cast<double>(forward_mm_s * dt) * std::cos(yaw_rad);
    y_mm += static_cast<double>(forward_mm_s * dt) * std::sin(yaw_rad);
    yaw_rad += static_cast<double>((speed[1] - speed[0]) / lf::kTrackCountsPerRad * dt);
    brick_hit = brick_hit || touches_brick();
  }

  // Any point of the footprint's outline inside the brick.
  [[nodiscard]] bool touches_brick() const {
    if (!has_brick) {
      return false;
    }
    const double c = std::cos(yaw_rad);
    const double s = std::sin(yaw_rad);
    const double half = 0.5 * static_cast<double>(lf::kBrickSizeMm);
    constexpr int kSteps = 20;
    for (int i = 0; i <= kSteps; ++i) {
      const double u = static_cast<double>(i) / kSteps;
      const double along = -kRearMm + u * (kFrontMm + kRearMm);
      const double across = -kHalfWidthMm + u * 2.0 * kHalfWidthMm;
      const std::array<std::array<double, 2>, 4> outline = {
        {{along, kHalfWidthMm}, {along, -kHalfWidthMm}, {-kRearMm, across}, {kFrontMm, across}}};
      for (const auto& p : outline) {
        const double x = x_mm + p[0] * c - p[1] * s;
        const double y = y_mm + p[0] * s + p[1] * c;
        if (x >= brick_front_x && x <= brick_front_x + 2.0 * half && std::abs(y) <= half) {
          return true;
        }
      }
    }
    return false;
  }

  [[nodiscard]] float read_eye(std::size_t eye) {
    if (saturated[eye]) {
      return 4095.0F;
    }
    const float forward_x = std::cos(static_cast<float>(yaw_rad));
    const float forward_y = std::sin(static_cast<float>(yaw_rad));
    // Eye 0 (D1) is the far left one.
    const float left = (3.5F - static_cast<float>(eye)) * kEyePitchMm;
    const float x = static_cast<float>(x_mm) + kBarAheadMm * forward_x - left * forward_y;
    const float y = static_cast<float>(y_mm) + kBarAheadMm * forward_y + left * forward_x;
    float nearest = 1.0e9F;
    for (const Segment& segment : course) {
      nearest = std::min(nearest, distance_to_segment(segment, x, y));
    }
    bool inside_square = false;
    for (const auto& square : dark_squares) {
      if (std::abs(x - square[0]) <= square[2] && std::abs(y - square[1]) <= square[2]) {
        inside_square = true;
        break;
      }
    }
    // Full line inside half the width, fading out over 3 mm; a node square is full dark.
    const float nearest_effective = inside_square ? 0.0F : nearest;
    const float cover =
      std::clamp((0.5F * kLineWidthMm + 1.5F - nearest_effective) / 3.0F, 0.0F, 1.0F);
    noise_state = noise_state * 1664525U + 1013904223U;
    const float noise = static_cast<float>((noise_state >> 24U) % 17U) - 8.0F;
    const float level =
      surface.floor[eye] + (surface.line[eye] - surface.floor[eye]) * cover + noise;
    return std::clamp(level, 0.0F, 4095.0F);
  }

  [[nodiscard]] lf::Measurements measure(std::uint64_t now_us) {
    lf::Measurements m;
    m.encoders.status = lf::SampleStatus::kValid;
    m.encoders.latest_us = now_us;
    m.encoders.earliest_us = now_us;
    for (std::size_t wheel = 0; wheel < lf::kWheelCount; ++wheel) {
      const auto forward = static_cast<std::int32_t>(std::llround(counts[wheel]));
      m.encoders.value[wheel] = lf::kInvertEncoder[wheel] ? -forward : forward;
    }
    if (gyro_alive) {
      ++sequence;
    }
    m.gyro_z_rad_s.status = lf::SampleStatus::kValid;
    m.gyro_z_rad_s.sequence = sequence;
    m.gyro_z_rad_s.latest_us = gyro_alive ? now_us : 0;
    const float rate = (speed[1] - speed[0]) / lf::kTrackCountsPerRad;
    m.gyro_z_rad_s.value = gyro_sign * rate + kGyroBiasRadS;
    m.battery_v.status = lf::SampleStatus::kValid;
    m.battery_v.value = battery_v;
    m.distance_mm.status = lf::SampleStatus::kValid;
    m.distance_mm.value = 500.0F;
    if (has_brick) {
      const double dx = brick_front_x - x_mm;
      const double dy = -y_mm;  // the brick sits on the line, at y = 0
      const double along = dx * std::cos(yaw_rad) + dy * std::sin(yaw_rad);
      const double cross = -dx * std::sin(yaw_rad) + dy * std::cos(yaw_rad);
      if (along > 0.0 && std::abs(cross) < 40.0) {
        m.distance_mm.value = static_cast<float>(along) - kBarAheadMm;
      }
    }
    m.distance_mm.latest_us = now_us - tof_age_us;
    m.ir.status = lf::SampleStatus::kValid;
    m.ir.latest_us = now_us;
    for (std::size_t eye = 0; eye < lf::kIrCount; ++eye) {
      m.ir.value[eye] = static_cast<std::uint16_t>(read_eye(eye));
    }
    return m;
  }
};

// Robot and plant wired together at the control rate.
struct Bench {
  static constexpr std::uint64_t kTickUs = 1000000 / lf::kControlHz;
  lf::Robot robot;
  Plant plant;
  std::uint64_t now_us = 1000000;
  lf::Outputs outputs;

  void tick(bool button = false) {
    now_us += kTickUs;
    outputs = robot.step(plant.measure(now_us), button, now_us);
    plant.advance(outputs, lf::kControlDt);
  }
  void run_ms(unsigned ms) {
    for (unsigned i = 0; i < ms / 2; ++i) {
      tick();
    }
  }
  // Held past the debounce time, then released.
  void press() {
    for (int i = 0; i < 40; ++i) {
      tick(true);
    }
    for (int i = 0; i < 40; ++i) {
      tick(false);
    }
  }
  [[nodiscard]] lf::Phase phase() const { return robot.snapshot().phase; }
  // Runs until the robot reaches `wanted` or stops; false if time ran out first.
  bool run_until(lf::Phase wanted, float limit_s) {
    const auto ticks = static_cast<int>(limit_s * static_cast<float>(lf::kControlHz));
    for (int i = 0; i < ticks; ++i) {
      tick();
      const lf::Phase now = phase();
      if (now == wanted || now == lf::Phase::kStopped || now == lf::Phase::kFinished) {
        return now == wanted;
      }
    }
    return false;
  }
  // Powered up, one press to calibrate, a second to race.
  bool start_race() {
    run_ms(100);
    press();
    if (!run_until(lf::Phase::kReady, 60.0F)) {
      return false;
    }
    press();
    return phase() == lf::Phase::kRacing;
  }
};
