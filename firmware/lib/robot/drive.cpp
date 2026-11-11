#include "drive.hpp"

#include <algorithm>
#include <cmath>

namespace lf {
namespace {

constexpr std::uint32_t kStallTicks = static_cast<std::uint32_t>(kStallUs * kControlHz / 1000000);
// Duty scales with the voltage the model was measured at; keep the correction mild.
constexpr float kMinVoltageScale = 0.8F;
constexpr float kMaxVoltageScale = 1.4F;

}  // namespace

void Drive::reset() {
  target_cps_.fill(0.0F);
  integral_.fill(0.0F);
  stall_ticks_.fill(0);
}

bool Drive::stalled() const {
  return std::any_of(stall_ticks_.begin(), stall_ticks_.end(),
                     [](std::uint32_t ticks) { return ticks >= kStallTicks; });
}

MotorOutput Drive::step(const Motion& motion, float v_cps, float omega_rad_s,
                        const Sample<float>& battery) {
  const float half_difference = 0.5F * kTrackCountsPerRad * omega_rad_s;
  target_cps_[0] = v_cps - half_difference;
  target_cps_[1] = v_cps + half_difference;

  float voltage_scale = 1.0F;
  if (battery.status == SampleStatus::kValid && battery.value > 1.0F) {
    voltage_scale = std::clamp(kModelVoltage / battery.value, kMinVoltageScale, kMaxVoltageScale);
  }

  MotorOutput output;
  output.mode = MotorMode::kDrive;
  for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
    const float target = target_cps_[wheel];
    const float measured = motion.wheel_cps(wheel);
    if (std::abs(target) < kMinTargetCps) {
      integral_[wheel] = 0.0F;
      stall_ticks_[wheel] = 0;
      output.duty[wheel] = 0.0F;
      continue;
    }
    // Stall: asked for real speed, getting a fraction of it.
    if (std::abs(target) >= kStallTargetCps &&
        std::abs(measured) < kStallRatio * std::abs(target)) {
      ++stall_ticks_[wheel];
    } else {
      stall_ticks_[wheel] = 0;
    }
    // IMC tuning with lambda = tau / speedup: kp = speedup / gain, ki = kp / tau.
    const float kp = kSpeedLoopSpeedup / kMotorGain[wheel];
    const float ki = kp / kMotorTau[wheel];
    const float sign = target > 0.0F ? 1.0F : -1.0F;
    const float feedforward =
      voltage_scale * (sign * kMotorDead[wheel] + target / kMotorGain[wheel]);
    const float error = target - measured;
    const float unsaturated = feedforward + kp * error + integral_[wheel];
    const float duty = std::clamp(unsaturated, -kMaxDuty, kMaxDuty);
    // Back-calculation takes whatever the clamp cut off back out of the integral,
    // and the trim limit stops the integral swallowing the feedforward.
    integral_[wheel] += (ki * error + (duty - unsaturated) / kMotorTau[wheel]) * kControlDt;
    integral_[wheel] = std::clamp(integral_[wheel], -kTrimLimitDuty, kTrimLimitDuty);
    output.duty[wheel] = duty;
  }
  return output;
}

}  // namespace lf
