#include "race.hpp"

#include <algorithm>
#include <cmath>

namespace lf {

float follow_turn(float position, float position_rate) {
  // Positive position is to the right; a positive turn is to the left. A drift to
  // the right must also turn right, so its term carries the same sign.
  return std::clamp(-kSteerDutyPerEye * position - kSteerDutyDamping * position_rate, -kTurnDutyMax,
                    kTurnDutyMax);
}

float follow_forward(float position, float position_rate) {
  const float activity =
    kSlowByPosition * std::abs(position) + kSlowByRate * std::abs(position_rate);
  return kCruiseDuty * std::clamp(1.0F - activity, kMinFollowFraction, 1.0F);
}

void Race::begin(const Motion& motion, std::uint64_t now_us) {
  state_ = RaceState::kScan;
  outcome_ = StopReason::kNone;
  scan_.reset();
  scanned_ = {};
  calibration_ = {};
  view_ = {};
  heading_start_ = motion.psi_rad();
  swing_ = 0;
  last_position_ = 0.0F;
  last_turn_ = 0.0F;
  approach_side_ = -1.0F;
  in_dark_ = false;
  begin_swing(motion, now_us);
}

void Race::fail(StopReason reason) {
  state_ = RaceState::kFailed;
  outcome_ = reason;
}

void Race::begin_swing(const Motion& motion, std::uint64_t now_us) {
  const float goal = heading_start_ + kScanSwingsRad[swing_];
  maneuver_.begin_turn(goal - motion.psi_rad(), motion, now_us, kScanTurnDuty);
}

void Race::update_view(const Measurements& measurements) {
  if (measurements.ir.status != SampleStatus::kValid) {
    return;
  }
  const std::array<float, kIrCount> dark = darkness(calibration_, measurements.ir.value);
  view_ = read_line(dark, last_position_);
  adapt_floor(calibration_, scanned_, measurements.ir.value, dark);
}

void Race::go() {
  if (state_ == RaceState::kReady) {
    last_position_ = 0.0F;
    enter_follow();
  }
}

Target Race::step(const Measurements& measurements, const Motion& motion, std::uint64_t now_us) {
  if (state_ == RaceState::kScan) {
    return step_scan(measurements, motion, now_us);
  }
  if (state_ == RaceState::kReturn) {
    return step_return(motion, now_us);
  }
  update_view(measurements);
  switch (state_) {
    case RaceState::kFollow:
      return step_follow(measurements, motion, now_us);
    case RaceState::kCorner:
      return step_corner(motion, now_us);
    case RaceState::kConfirmGo:
      return step_confirm_go(motion, now_us);
    case RaceState::kBypass:
      return step_bypass(motion, now_us);
    case RaceState::kScan:
    case RaceState::kReturn:
    case RaceState::kReady:
    case RaceState::kFinished:
    case RaceState::kFailed:
      break;
  }
  return {};
}

Target Race::step_scan(const Measurements& measurements, const Motion& motion,
                       std::uint64_t now_us) {
  if (measurements.ir.status == SampleStatus::kValid) {
    scan_.add(measurements.ir.value);
  }
  const Target target = maneuver_.step(motion, now_us);
  if (!maneuver_.done()) {
    return target;
  }
  ++swing_;
  const LineCalibration found = scan_.calibration();
  const bool all_eyes = found.usable_count == kIrCount;
  if (!all_eyes && swing_ < kScanSwingsRad.size()) {
    begin_swing(motion, now_us);
    return {};
  }
  if (!found.valid()) {
    fail(StopReason::kScanFailed);
    return {};
  }
  scanned_ = found;
  calibration_ = found;
  edge_position_ = std::max(outermost_usable_position(found) - kEdgeInsetEyes, 1.0F);
  state_ = RaceState::kReturn;
  maneuver_.begin_turn(heading_start_ - motion.psi_rad(), motion, now_us, kScanTurnDuty);
  return {};
}

Target Race::step_return(const Motion& motion, std::uint64_t now_us) {
  const Target target = maneuver_.step(motion, now_us);
  if (maneuver_.done()) {
    state_ = RaceState::kReady;
    return {};
  }
  return target;
}

void Race::enter_follow() {
  state_ = RaceState::kFollow;
  in_dark_ = false;
}

void Race::enter_corner(float side, const Motion& motion, std::uint64_t now_us) {
  state_ = RaceState::kCorner;
  corner_side_ = side;
  corner_start_psi_ = motion.psi_rad();
  corner_stage_ = 0;
  // Roll forward to put the wheels at the corner, then probe.
  maneuver_.begin_go(kCornerAdvanceMm, motion, now_us);
}

void Race::enter_bypass(const Motion& motion, std::uint64_t /*now_us*/) {
  state_ = RaceState::kBypass;
  bypass_stage_ = 0;
  bypass_start_psi_ = motion.psi_rad();
  braking_ = true;
}

Target Race::step_follow(const Measurements& measurements, const Motion& motion,
                         std::uint64_t now_us) {
  // A brick in front ends the line-following until it has been gone around.
  const Sample<float>& distance = measurements.distance_mm;
  const bool tof_fresh =
    distance.status == SampleStatus::kValid && now_us - distance.latest_us <= kTofStaleUs;
  if (tof_fresh && distance.value < kBlockApproachMm) {
    enter_bypass(motion, now_us);
    return {};
  }
  switch (view_.shape) {
    case LineShape::kLine: {
      in_dark_ = false;
      const float position = view_.position;
      // How fast the offset is moving, low-passed: the derivative reacts to the
      // corner before the line reaches the edge of the bar.
      if (last_view_us_ != 0 && now_us > last_view_us_) {
        const float dt = static_cast<float>(now_us - last_view_us_) / 1.0e6F;
        const float instant = (position - last_position_) / dt;
        position_rate_ += (instant - position_rate_) * std::min(1.0F, dt * kRateFilterPerS);
      }
      last_view_us_ = now_us;
      last_position_ = position;
      approach_side_ = position <= -0.3F ? 1.0F : -1.0F;
      last_turn_ = follow_turn(position, position_rate_);
      return {follow_forward(position, position_rate_), last_turn_};
    }
    case LineShape::kWide:
    case LineShape::kSplit: {
      if (!in_dark_) {
        in_dark_ = true;
        dark_start_s_ = motion.s_counts();
      } else if ((motion.s_counts() - dark_start_s_) * kMmPerCount > kNodeStraightMm) {
        fail(StopReason::kWideTooLong);
        return {};
      }
      return {kNodeCrawlDuty, last_turn_};
    }
    case LineShape::kNone:
      break;
  }
  in_dark_ = false;
  enter_corner(approach_side_, motion, now_us);
  return {};
}

// Roll to the corner, then probe for the exit line: first the side the line was
// drifting towards, then the other, then straight back to the start heading. A
// line found anywhere in that arc is followed; none means the line really did end,
// so roll on a little to confirm the finish rather than declare it from a guess.
Target Race::step_corner(const Motion& motion, std::uint64_t now_us) {
  if (corner_stage_ > 0 && view_.shape == LineShape::kLine) {
    enter_follow();
    last_position_ = view_.position;
    return {};
  }
  const Target target = maneuver_.step(motion, now_us);
  if (!maneuver_.done()) {
    return target;
  }
  ++corner_stage_;
  if (corner_stage_ == 1) {
    maneuver_.begin_turn(corner_side_ * kProbeRad, motion, now_us, kTurnDutyMax);
    return {};
  }
  if (corner_stage_ == 2) {
    maneuver_.begin_turn(corner_start_psi_ - corner_side_ * kProbeRad - motion.psi_rad(), motion,
                         now_us, kTurnDutyMax);
    return {};
  }
  if (corner_stage_ == 3) {
    maneuver_.begin_turn(corner_start_psi_ - motion.psi_rad(), motion, now_us, kTurnDutyMax);
    return {};
  }
  state_ = RaceState::kConfirmGo;
  maneuver_.begin_go(kConfirmMm, motion, now_us);
  return {};
}

Target Race::step_confirm_go(const Motion& motion, std::uint64_t now_us) {
  if (view_.shape == LineShape::kLine) {
    enter_follow();
    return {};
  }
  const Target target = maneuver_.step(motion, now_us);
  if (!maneuver_.done()) {
    return target;
  }
  state_ = RaceState::kFinished;
  outcome_ = StopReason::kFinish;
  return {};
}

// Fixed right-hand detour around a brick: brake off the line speed, turn right,
// pass the brick's width, turn left, pass its length, turn left, drive back towards
// the line, then face along it again before following. The last alignment matters:
// the robot reaches the line heading across it, and following from that heading
// would drive straight off the far side.
Target Race::step_bypass(const Motion& motion, std::uint64_t now_us) {
  if (braking_) {
    const float forward_cps = 0.5F * (motion.wheel_cps(0) + motion.wheel_cps(1));
    if (std::abs(forward_cps) < kBlockStopCps) {
      braking_ = false;
      // Turn right, absolutely: to the line heading minus a quarter turn.
      maneuver_.begin_turn(bypass_start_psi_ - kBlockTurnRad - motion.psi_rad(), motion, now_us,
                           kBlockTurnDuty);
      return {};
    }
    return {-kBlockBrakeDuty, 0.0F};  // drive against the momentum until it stops
  }
  if (bypass_stage_ == 5 && view_.shape != LineShape::kNone) {
    // The search leg found the line (as a dark band, since it crosses it head-on);
    // turn to face along it before following.
    bypass_stage_ = 6;
    maneuver_.begin_turn(bypass_start_psi_ - motion.psi_rad(), motion, now_us, kBlockTurnDuty);
    return {};
  }
  if (bypass_stage_ == 6) {
    if (maneuver_.done()) {
      enter_follow();
      last_position_ = view_.position;
    }
    return maneuver_.step(motion, now_us);
  }
  const Target target = maneuver_.step(motion, now_us);
  if (!maneuver_.done()) {
    return target;
  }
  ++bypass_stage_;
  switch (bypass_stage_) {
    case 1:
      maneuver_.begin_go(kBlockSideClearMm, motion, now_us);
      break;
    case 2:
      maneuver_.begin_turn(bypass_start_psi_ - motion.psi_rad(), motion, now_us, kBlockTurnDuty);
      break;
    case 3:
      maneuver_.begin_go(kBlockLengthClearMm, motion, now_us);
      break;
    case 4:
      maneuver_.begin_turn(bypass_start_psi_ + kBlockTurnRad - motion.psi_rad(), motion, now_us,
                           kBlockTurnDuty);
      break;
    case 5:
      maneuver_.begin_go(kBlockSearchMm, motion, now_us);
      break;
    default:
      fail(StopReason::kBypassLost);
      break;
  }
  return {};
}

}  // namespace lf
