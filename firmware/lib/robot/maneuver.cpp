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
}

void Maneuver::begin_turn(float rad, const Motion& motion, std::uint64_t now_us) {
  kind_ = Kind::kTurn;
  done_ = false;
  start_us_ = now_us;
  goal_ = rad;
  psi_start_ = motion.psi_rad();
}

bool Maneuver::timed_out(std::uint64_t now_us) const {
  return now_us - start_us_ > kManeuverTimeoutUs;
}

Target Maneuver::step(const Motion& motion, std::uint64_t now_us) {
  return kind_ == Kind::kGo ? step_go(motion, now_us) : step_turn(motion, now_us);
}

Target Maneuver::step_go(const Motion& motion, std::uint64_t now_us) {
  const float direction = goal_ >= 0.0F ? 1.0F : -1.0F;
  const float remaining = direction * (goal_ - (motion.s_counts() - s_start_));
  // Arriving or passing the goal both end the move: braking lag means a robot
  // that is merely near it would creep back and forth otherwise.
  if (remaining <= kGoToleranceMm / kMmPerCount) {
    done_ = true;
    return {};
  }
  const float seconds = static_cast<float>(now_us - start_us_) / kMicrosecondsPerSecond;
  const float ramp = kGoMinCps + kGoAccelCpsPerS * seconds;
  const float approach = kGoApproachPerS * remaining;
  const float speed = std::clamp(std::min({kGoCps, ramp, approach}), kGoMinCps, kGoCps);
  Target target;
  target.v_cps = direction * speed;
  target.omega_rad_s = std::clamp(kHeadingHoldPerS * (psi_start_ - motion.psi_rad()),
                                  -kHoldOmegaMaxRadS, kHoldOmegaMaxRadS);
  return target;
}

Target Maneuver::step_turn(const Motion& motion, std::uint64_t /*now_us*/) {
  const float direction = goal_ >= 0.0F ? 1.0F : -1.0F;
  const float remaining = direction * (goal_ - (motion.psi_rad() - psi_start_));
  // Arriving or passing the goal both end the turn.
  if (remaining <= kTurnToleranceRad) {
    done_ = true;
    return {};
  }
  Target target;
  target.omega_rad_s =
    direction * std::clamp(kTurnGainPerS * remaining, kTurnOmegaMinRadS, kTurnOmegaMaxRadS);
  return target;
}

}  // namespace lf
