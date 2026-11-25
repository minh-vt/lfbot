#pragma once

#include <cstdint>

#include "config.hpp"
#include "line.hpp"
#include "motion.hpp"
#include "types.hpp"

namespace lf {

// The steering law, one for every situation. position is in eye pitches (positive =
// line to the right); position_rate is eye/s. Pure, so its shape is testable.
[[nodiscard]] float follow_turn(float position, float position_rate);
[[nodiscard]] float follow_forward(float position, float position_rate);

// Turn duty that brings a heading error to zero, for turning in place.
[[nodiscard]] float turn_toward(float error_rad, float rate_rad_s);

// A line drawn in memory around a brick on the right (ADR-026). It lives in the
// frame of the real line where the brick was seen: `along` runs down that line,
// `left` is to its left. The robot follows it with the same law as a real line, by
// asking what the bar would read if this line were on the floor.
class Detour {
public:
  // brick_mm: from the axle to the brick's face, along the line.
  void plan(const Motion& motion, float brick_mm);
  void end() { active_ = false; }
  [[nodiscard]] bool active() const { return active_; }
  [[nodiscard]] float position_at_bar(const Motion& motion) const;
  // The real line may be taken again: the bar is past the brick and back near the
  // old line's axis. Anything dark elsewhere is another part of the track.
  [[nodiscard]] bool may_rejoin(const Motion& motion) const;
  [[nodiscard]] bool overrun(const Motion& motion) const;

private:
  struct Place {
    float along = 0.0F;
    float left = 0.0F;
  };
  [[nodiscard]] Place bar(const Motion& motion) const;
  [[nodiscard]] float offset(float along) const;

  bool active_ = false;
  float x_mm_ = 0.0F;
  float y_mm_ = 0.0F;
  float psi_rad_ = 0.0F;
  float brick_back_ = 0.0F;
  float rise_start_ = 0.0F;
  float fall_start_ = 0.0F;
};

// Steers the robot round the course: learns the bar by swinging over the line, then
// follows whatever guides it this tick, the real line or a virtual one. Returns a
// forward and a turn demand; the wheels are the drive's business.
class Pilot {
public:
  void begin(const Motion& motion);
  [[nodiscard]] Target calibrate(const Measurements& measurements, const Motion& motion);
  [[nodiscard]] bool calibrated() const { return calibrated_; }
  void go();
  // pace: how many times run 1's duty to follow a plain line (Course::pace).
  [[nodiscard]] Target race(const Measurements& measurements, const Motion& motion,
                            std::uint64_t now_us, float pace);

  // kNone while all is well; kFinish at the end of the line; otherwise why it failed.
  [[nodiscard]] StopReason outcome() const { return outcome_; }
  [[nodiscard]] Guide guide() const { return guide_; }
  [[nodiscard]] const LineView& view() const { return view_; }
  [[nodiscard]] const LineCalibration& calibration() const { return calibration_; }

private:
  void update_view(const Measurements& measurements);
  void watch_for_brick(const Measurements& measurements, const Motion& motion,
                       std::uint64_t now_us);
  [[nodiscard]] float aim(const Motion& motion);
  [[nodiscard]] float follow_line();
  [[nodiscard]] float hold_through_dark(const Motion& motion);
  [[nodiscard]] float search(const Motion& motion);
  [[nodiscard]] float follow_detour(const Motion& motion);
  void track_rate(float position, Guide previous, std::uint64_t now_us);
  [[nodiscard]] float forward_for(float position, const Motion& motion) const;

  StopReason outcome_ = StopReason::kNone;
  Guide guide_ = Guide::kLine;
  LineScan scan_;
  LineCalibration scanned_;      // what the scan found; the floor level may drift from it
  LineCalibration calibration_;  // what is in use now
  LineView view_;
  Detour detour_;

  float heading_start_ = 0.0F;
  std::size_t swing_ = 0;
  bool calibrated_ = false;

  float last_position_ = 0.0F;  // the last aim taken from a real line
  float side_ = 1.0F;           // which side the line was last off-centre: +1 right
  float position_rate_ = 0.0F;  // eye/s, low-passed
  float rate_position_ = 0.0F;
  std::uint64_t rate_us_ = 0;

  float pace_ = 1.0F;

  bool in_dark_ = false;
  float dark_start_mm_ = 0.0F;
  bool lost_ = false;
  float lost_mm_ = 0.0F;  // where the line went, or where the straight check began
  float lost_psi_ = 0.0F;
  float search_side_ = 0.0F;  // 0 once both sides are searched
  int searched_sides_ = 0;
};

}  // namespace lf
