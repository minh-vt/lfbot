#include "pilot.hpp"

#include <algorithm>
#include <cmath>

namespace lf {
namespace {

constexpr float kMicrosecondsPerSecond = 1.0e6F;
// Off-centre by at least this many eyes, the line counts as being on one side.
constexpr float kSideEyes = 0.3F;

float smooth_step(float u) {
  const float t = std::clamp(u, 0.0F, 1.0F);
  return t * t * (3.0F - 2.0F * t);
}

float sign_of(float value) {
  return value < 0.0F ? -1.0F : 1.0F;
}

}  // namespace

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

float turn_toward(float error_rad, float rate_rad_s) {
  const float push = kTurnPPerRad * error_rad - kTurnDDutyPerRate * rate_rad_s;
  const float least = std::abs(error_rad) > kTurnToleranceRad ? kTurnMinDuty : 0.0F;
  return std::copysign(std::clamp(std::abs(push), least, kTurnDutyMax), push);
}

// --- Detour --------------------------------------------------------------------

void Detour::plan(const Motion& motion, float brick_mm) {
  active_ = true;
  x_mm_ = motion.x_mm();
  y_mm_ = motion.y_mm();
  psi_rad_ = motion.psi_rad();
  brick_back_ = brick_mm + kBrickSizeMm;
  // Side-step as soon as the bar can, and be fully out before the brick's face.
  rise_start_ = std::max(kBarAheadMm, brick_mm - kDetourLeadMm - kDetourRampMm);
  fall_start_ = brick_back_ + kDetourTailMm;
}

Detour::Place Detour::bar(const Motion& motion) const {
  const float bar_x = motion.x_mm() + kBarAheadMm * std::cos(motion.psi_rad()) - x_mm_;
  const float bar_y = motion.y_mm() + kBarAheadMm * std::sin(motion.psi_rad()) - y_mm_;
  const float c = std::cos(psi_rad_);
  const float s = std::sin(psi_rad_);
  return {bar_x * c + bar_y * s, -bar_x * s + bar_y * c};
}

// Left of the old line, in mm: negative is the right-hand side of the brick.
float Detour::offset(float along) const {
  return -kDetourSideMm * (smooth_step((along - rise_start_) / kDetourRampMm) -
                           smooth_step((along - fall_start_) / kDetourRampMm));
}

float Detour::position_at_bar(const Motion& motion) const {
  const Place place = bar(motion);
  // The virtual line is this far to the bar's left, seen across a bar that may be
  // turned away from the old line.
  const float left_mm = (offset(place.along) - place.left) * std::cos(motion.psi_rad() - psi_rad_);
  return std::clamp(-left_mm / kEyePitchMm, -kSearchAimEyes, kSearchAimEyes);
}

bool Detour::may_rejoin(const Motion& motion) const {
  const Place place = bar(motion);
  return place.along >= brick_back_ && std::abs(place.left) <= kRejoinCorridorMm;
}

bool Detour::overrun(const Motion& motion) const {
  return bar(motion).along > fall_start_ + kDetourRampMm + kDetourSearchMm;
}

// --- Calibration -----------------------------------------------------------------

void Pilot::begin(const Motion& motion) {
  outcome_ = StopReason::kNone;
  scan_.reset();
  scanned_ = {};
  calibration_ = {};
  view_ = {};
  heading_start_ = motion.psi_rad();
  swing_ = 0;
  calibrated_ = false;
}

// Swing left and right, wider each time, until every eye has seen floor and line;
// then face the start heading again.
Target Pilot::calibrate(const Measurements& measurements, const Motion& motion) {
  if (swing_ < kScanSwingsRad.size()) {
    if (measurements.ir.status == SampleStatus::kValid) {
      scan_.add(measurements.ir.value);
    }
    const float goal = kScanSwingsRad[swing_];
    const float turned = motion.psi_rad() - heading_start_;
    if (goal > 0.0F ? turned < goal : turned > goal) {
      return {0.0F, std::copysign(kScanTurnDuty, goal)};
    }
    const LineCalibration found = scan_.calibration();
    swing_ = found.usable_count == kIrCount ? kScanSwingsRad.size() : swing_ + 1;
    if (swing_ < kScanSwingsRad.size()) {
      return {};
    }
    if (!found.valid()) {
      outcome_ = StopReason::kScanFailed;
      return {};
    }
    scanned_ = found;
    calibration_ = found;
  }
  const float error = heading_start_ - motion.psi_rad();
  if (std::abs(error) < kTurnToleranceRad &&
      std::abs(motion.yaw_rate_rad_s()) < kTurnSettledRateRadS) {
    calibrated_ = true;
    return {};
  }
  return {0.0F, turn_toward(error, motion.yaw_rate_rad_s())};
}

// --- Racing -----------------------------------------------------------------------

void Pilot::go() {
  guide_ = Guide::kLine;
  last_position_ = 0.0F;
  side_ = 1.0F;
  position_rate_ = 0.0F;
  rate_us_ = 0;
  in_dark_ = false;
  lost_ = false;
  detour_.end();
}

void Pilot::update_view(const Measurements& measurements) {
  if (measurements.ir.status != SampleStatus::kValid) {
    return;
  }
  const std::array<float, kIrCount> dark = darkness(calibration_, measurements.ir.value);
  view_ = read_line(dark, last_position_);
  adapt_floor(calibration_, scanned_, measurements.ir.value, dark);
}

// Only a brick seen while plainly following the line starts a detour: what the ToF
// sees while the robot is turning about is the rest of the course.
void Pilot::watch_for_brick(const Measurements& measurements, const Motion& motion,
                            std::uint64_t now_us) {
  const Sample<float>& distance = measurements.distance_mm;
  const bool fresh =
    distance.status == SampleStatus::kValid && now_us - distance.latest_us <= kTofStaleUs;
  if (detour_.active() || guide_ != Guide::kLine || !fresh || distance.value >= kBrickSeeMm) {
    return;
  }
  // The ToF sits at the front, level with the bar.
  detour_.plan(motion, distance.value + kBarAheadMm);
}

Target Pilot::race(const Measurements& measurements, const Motion& motion, std::uint64_t now_us,
                   float pace) {
  pace_ = pace;
  update_view(measurements);
  watch_for_brick(measurements, motion, now_us);
  const Guide previous = guide_;
  const float position = aim(motion);
  if (outcome_ != StopReason::kNone) {
    return {};
  }
  track_rate(position, previous, now_us);
  // Checking for the end is facing the old heading in place, then rolling on; every
  // other guide is a line position for the follow law.
  const float turn = guide_ == Guide::kStraight
                       ? turn_toward(lost_psi_ - motion.psi_rad(), motion.yaw_rate_rad_s())
                       : follow_turn(position, position_rate_);
  return {forward_for(position, motion), turn};
}

// Where to steer this tick. Every case returns a line position for the same law;
// they differ only in where that position comes from.
float Pilot::aim(const Motion& motion) {
  if (detour_.active()) {
    return follow_detour(motion);
  }
  if (view_.shape == LineShape::kLine) {
    return follow_line();
  }
  if (view_.shape != LineShape::kNone) {
    return hold_through_dark(motion);
  }
  return search(motion);
}

float Pilot::follow_line() {
  guide_ = Guide::kLine;
  in_dark_ = false;
  lost_ = false;
  last_position_ = view_.position;
  if (std::abs(view_.position) >= kSideEyes) {
    side_ = sign_of(view_.position);
  }
  return view_.position;
}

// A crossing, a node or a thick mark: keep the aim, so a circle that crosses itself
// keeps its curve instead of turning onto the other arc.
float Pilot::hold_through_dark(const Motion& motion) {
  guide_ = Guide::kMemory;
  lost_ = false;
  const float s_mm = motion.s_counts() * kMmPerCount;
  if (!in_dark_) {
    in_dark_ = true;
    dark_start_mm_ = s_mm;
  } else if (s_mm - dark_start_mm_ > kNodeStraightMm) {
    outcome_ = StopReason::kWideTooLong;
  }
  return last_position_;
}

// The line is gone: roll until the axle is where the bar lost it, then turn towards
// the side it was last off-centre, then the other. Found on neither side, it ended:
// face the old heading and roll on a little; still nothing is the finish.
float Pilot::search(const Motion& motion) {
  const float s_mm = motion.s_counts() * kMmPerCount;
  if (!lost_) {
    lost_ = true;
    lost_mm_ = s_mm;
    lost_psi_ = motion.psi_rad();
    searched_sides_ = 0;
    search_side_ = side_;
    in_dark_ = false;
  }
  if (search_side_ != 0.0F) {
    guide_ = Guide::kSearch;
    if (s_mm - lost_mm_ < kBarAheadMm) {
      return (motion.psi_rad() - lost_psi_) * kHoldEyesPerRad;
    }
    const float turned = -search_side_ * (motion.psi_rad() - lost_psi_);
    if (turned < kSearchTurnRad) {
      return search_side_ * kSearchAimEyes;
    }
    if (++searched_sides_ < 2) {
      search_side_ = -search_side_;
      return search_side_ * kSearchAimEyes;
    }
    search_side_ = 0.0F;
    lost_mm_ = s_mm;
  }
  guide_ = Guide::kStraight;
  if (s_mm - lost_mm_ > kConfirmMm) {
    outcome_ = StopReason::kFinish;
  }
  return 0.0F;
}

float Pilot::follow_detour(const Motion& motion) {
  if (view_.shape == LineShape::kLine && detour_.may_rejoin(motion)) {
    detour_.end();
    return follow_line();
  }
  if (detour_.overrun(motion)) {
    outcome_ = StopReason::kLineLost;
  }
  guide_ = Guide::kVirtual;
  return detour_.position_at_bar(motion);
}

// How fast the aim is moving, low-passed; only meaningful while it follows a line,
// real or virtual, without jumping from one source to another.
void Pilot::track_rate(float position, Guide previous, std::uint64_t now_us) {
  const bool following = guide_ == Guide::kLine || guide_ == Guide::kVirtual;
  if (!following || guide_ != previous || rate_us_ == 0 || now_us <= rate_us_) {
    position_rate_ = 0.0F;
  } else {
    const float dt = static_cast<float>(now_us - rate_us_) / kMicrosecondsPerSecond;
    const float instant = (position - rate_position_) / dt;
    position_rate_ += (instant - position_rate_) * std::min(1.0F, dt * kRateFilterPerS);
  }
  rate_position_ = position;
  rate_us_ = now_us;
}

float Pilot::forward_for(float position, const Motion& motion) const {
  switch (guide_) {
    case Guide::kLine:
      return follow_forward(position, position_rate_) * pace_;
    case Guide::kVirtual:
      return std::min(follow_forward(position, position_rate_), kDetourDuty);
    case Guide::kMemory:
      return kNodeCrawlDuty;
    case Guide::kStraight:
      // Face the old heading first, so the roll-on is straight.
      return std::abs(motion.psi_rad() - lost_psi_) < kTurnToleranceRad ? kNodeCrawlDuty : 0.0F;
    case Guide::kSearch:
      return motion.s_counts() * kMmPerCount - lost_mm_ < kBarAheadMm ? kNodeCrawlDuty : 0.0F;
  }
  return 0.0F;
}

}  // namespace lf
