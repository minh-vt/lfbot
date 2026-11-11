#pragma once

#include <array>
#include <cmath>
#include <cstdint>

#include "config.hpp"
#include "robot.hpp"

// A stand-in robot for closed-loop contract tests: two first-order motors, a
// body that turns by the wheel speed difference, a gyro and encoders reporting
// with the wiring signs of the real robot. The motor deliberately differs from
// the controller's model (weaker, wider dead zone) so the speed loop has work.
// It proves the signs and the plumbing, not that the real robot will behave.
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

  void advance(const lf::Outputs& outputs, float dt) {
    for (std::size_t wheel = 0; wheel < lf::kWheelCount; ++wheel) {
      float duty = outputs.motor.mode == lf::MotorMode::kDrive ? outputs.motor.duty[wheel] : 0.0F;
      if (lf::kInvertWheel[wheel]) {
        duty = -duty;
      }
      const float dead = lf::kMotorDead[wheel] + kExtraDead;
      const float magnitude = std::max(0.0F, std::abs(duty) - dead);
      const float steady =
        (duty >= 0.0F ? 1.0F : -1.0F) * magnitude * lf::kMotorGain[wheel] * kGainScale;
      speed[wheel] += (steady - speed[wheel]) * dt / lf::kMotorTau[wheel];
      if (jammed) {
        speed[wheel] = 0.0F;
      }
      counts[wheel] += static_cast<double>(speed[wheel] * dt);
    }
    yaw_rad += static_cast<double>((speed[1] - speed[0]) / lf::kTrackCountsPerRad * dt);
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
    m.distance_mm.latest_us = now_us - tof_age_us;
    m.ir.status = lf::SampleStatus::kValid;
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

  void tick(bool button = false, const lf::Command& command = {}) {
    now_us += kTickUs;
    outputs = robot.step(plant.measure(now_us), button, now_us, command);
    plant.advance(outputs, lf::kControlDt);
  }
  void run_ms(unsigned ms) {
    for (unsigned i = 0; i < ms / 2; ++i) {
      tick();
    }
  }
  // Runs until the robot leaves the running phases or the time is up.
  void run_until_stopped(unsigned max_ms) {
    for (unsigned i = 0; i < max_ms / 2; ++i) {
      tick();
      const lf::Phase phase = robot.snapshot().phase;
      if (phase == lf::Phase::kIdle || phase == lf::Phase::kStopped) {
        if (i > 10) {
          return;
        }
      }
    }
  }
};
