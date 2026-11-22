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
  stalled_ = false;
  stall_ticks_ = 0;
}

MotorOutput Drive::step(const Target& target, const Sample<float>& battery, const Motion& motion) {
  float scale = 1.0F;
  if (battery.status == SampleStatus::kValid && battery.value > 1.0F) {
    scale = std::clamp(kModelVoltage / battery.value, kMinVoltageScale, kMaxVoltageScale);
  }
  const float forward = target.forward * scale;
  const float turn = target.turn * scale;
  MotorOutput output;
  output.mode = MotorMode::kDrive;
  output.duty[0] = std::clamp(forward - turn, -kMaxDuty, kMaxDuty);  // left
  output.duty[1] = std::clamp(forward + turn, -kMaxDuty, kMaxDuty);  // right

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
