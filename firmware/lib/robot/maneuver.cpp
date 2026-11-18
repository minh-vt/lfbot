#include "maneuver.hpp"

#include <algorithm>
#include <cmath>

#include "config.hpp"

namespace lf {
namespace {

constexpr float kMicrosecondsPerSecond = 1.0e6F;

}  // namespace

void Maneuver::begin_go(float mm, const Motion& motion, std::uint64_t now_us) {
  kind_ = Kind::kGo;
  done_ = false;
  start_us_ = now_us;
  goal_ = mm / kMmPerCount;
  s_start_ = motion.s_counts();
  psi_start_ = motion.psi_rad();
  integral_ = 0.0F;
  heading_integral_ = 0.0F;
}

void Maneuver::begin_turn(float rad, const Motion& motion, std::uint64_t now_us, float turn_duty) {
  kind_ = Kind::kTurn;
  done_ = false;
  start_us_ = now_us;
  goal_ = rad;
  psi_start_ = motion.psi_rad();
  turn_duty_ = turn_duty;
  integral_ = 0.0F;
}

bool Maneuver::timed_out(std::uint64_t now_us) const {
  return now_us - start_us_ > kManeuverTimeoutUs;
}

Target Maneuver::step(const Motion& motion, std::uint64_t /*now_us*/) {
  return kind_ == Kind::kGo ? step_go(motion) : step_turn(motion);
}

Target Maneuver::step_go(const Motion& motion) {
  const float error = goal_ - (motion.s_counts() - s_start_);  // signed: positive still to go
  const float rate = 0.5F * (motion.wheel_cps(0) + motion.wheel_cps(1));
  if (std::abs(error) < kGoToleranceCounts && std::abs(rate) < kGoStopCps) {
    done_ = true;
    return {};
  }
  if (std::abs(error) < kGoIntegralWindowCounts) {
    integral_ += kGoIPerCountS * error * kControlDt;
    integral_ = std::clamp(integral_, -kGoDutyMax, kGoDutyMax);
  }
  const float forward =
    std::clamp(kGoPPerCount * error + integral_ - kGoDDutyPerCps * rate, -kGoDutyMax, kGoDutyMax);
  const float turn = std::clamp(kHoldDutyPerRad * (psi_start_ - motion.psi_rad()),
                                -kHoldTurnMaxDuty, kHoldTurnMaxDuty);
  return {forward, turn};
}

Target Maneuver::step_turn(const Motion& motion) {
  const float error = goal_ - (motion.psi_rad() - psi_start_);  // signed angle still to go
  const float rate = motion.yaw_rate_rad_s();
  if (std::abs(error) < kTurnToleranceRad && std::abs(rate) < kTurnSettledRateRadS) {
    done_ = true;
    return {};
  }
  if (std::abs(error) < kTurnIntegralWindowRad) {
    integral_ += kTurnIPerRadS * error * kControlDt;
    integral_ = std::clamp(integral_, -turn_duty_, turn_duty_);
  }
  const float turn = std::clamp(kTurnPPerRad * error + integral_ - kTurnDDutyPerRate * rate,
                                -turn_duty_, turn_duty_);
  return {0.0F, turn};
}

}  // namespace lf
