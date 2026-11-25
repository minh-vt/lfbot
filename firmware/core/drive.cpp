#include "drive.hpp"

#include <algorithm>
#include <cmath>

#include "config.hpp"

namespace lf {
namespace {

constexpr float kMinVoltageScale = 0.8F;
constexpr float kMaxVoltageScale = 1.4F;

}  // namespace

void Drive::reset() {
  forward_ = 0.0F;
  stalled_ = false;
  stall_ticks_ = 0;
}

MotorOutput Drive::step(const Target& target, const Sample<float>& battery, const Motion& motion) {
  float scale = 1.0F;
  if (battery.status == SampleStatus::kValid && battery.value > 1.0F) {
    scale = std::clamp(kModelVoltage / battery.value, kMinVoltageScale, kMaxVoltageScale);
  }
  const float turn = std::clamp(target.turn * scale, -kMaxDuty, kMaxDuty);
  const float wanted = std::clamp(target.forward * scale, forward_ - kForwardFallPerTick,
                                  forward_ + kForwardRisePerTick);
  const float room = kMaxDuty - std::abs(turn);
  forward_ = std::clamp(wanted, -room, room);
  MotorOutput output;
  output.mode = MotorMode::kDrive;
  output.duty[0] = forward_ - turn;  // left
  output.duty[1] = forward_ + turn;  // right

  // Jammed: asked to move forward, but neither wheel turns.
  const bool commanding = std::abs(target.forward) >= kStallForwardDuty;
  const bool moving =
    std::abs(motion.wheel_cps(0)) > kStallCps || std::abs(motion.wheel_cps(1)) > kStallCps;
  if (commanding && !moving) {
    if (++stall_ticks_ >= kStallTicks) {
      stalled_ = true;
    }
  } else {
    stall_ticks_ = 0;
  }
  return output;
}

}  // namespace lf
