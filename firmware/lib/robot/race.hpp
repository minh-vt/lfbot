#pragma once

#include <cstdint>

#include "config.hpp"
#include "line.hpp"
#include "maneuver.hpp"
#include "motion.hpp"
#include "types.hpp"

namespace lf {

// Steering and speed for following a line: pure functions so their shape is testable.
// position is in eye pitches (positive = line to the right); position_rate is eye/s.
[[nodiscard]] float follow_turn(float position, float position_rate);
[[nodiscard]] float follow_forward(float position, float position_rate);

// Decides what the robot does on the course: learn the line, follow it, turn
// corners, cross junctions, go around bricks, and recognise the end of the line.
// Returns a speed and a turn rate; the wheels are someone else's business.
class Race {
public:
  void begin(const Motion& motion, std::uint64_t now_us);
  [[nodiscard]] Target step(const Measurements& measurements, const Motion& motion,
                            std::uint64_t now_us);
  // Calibrated and waiting: start following. Does nothing in any other state.
  void go();

  [[nodiscard]] RaceState state() const { return state_; }
  // kNone while the race is on; kFinish at the end of the line; otherwise why it failed.
  [[nodiscard]] StopReason outcome() const { return outcome_; }
  [[nodiscard]] const LineView& view() const { return view_; }
  [[nodiscard]] const LineCalibration& calibration() const { return calibration_; }

private:
  void update_view(const Measurements& measurements);
  void begin_swing(const Motion& motion, std::uint64_t now_us);
  void enter_follow();
  void enter_corner(float side, const Motion& motion, std::uint64_t now_us);
  void enter_bypass(const Motion& motion, std::uint64_t now_us);
  void fail(StopReason reason);
  [[nodiscard]] Target step_scan(const Measurements& measurements, const Motion& motion,
                                 std::uint64_t now_us);
  [[nodiscard]] Target step_return(const Motion& motion, std::uint64_t now_us);
  [[nodiscard]] Target step_follow(const Measurements& measurements, const Motion& motion,
                                   std::uint64_t now_us);
  [[nodiscard]] Target step_corner(const Motion& motion, std::uint64_t now_us);
  [[nodiscard]] Target step_confirm_go(const Motion& motion, std::uint64_t now_us);
  [[nodiscard]] Target step_bypass(const Motion& motion, std::uint64_t now_us);

  RaceState state_ = RaceState::kFailed;
  StopReason outcome_ = StopReason::kNone;
  Maneuver maneuver_;
  LineScan scan_;
  LineCalibration scanned_;      // what the scan found; the floor level may drift from it
  LineCalibration calibration_;  // what is in use now
  LineView view_;

  float heading_start_ = 0.0F;
  std::size_t swing_ = 0;
  float last_position_ = 0.0F;
  float edge_position_ = 0.0F;   // beyond this a vanishing line left through the side
  float last_turn_ = 0.0F;       // last steering demand, in turn duty
  float approach_side_ = -1.0F;  // which way to probe first: -1 right, +1 left
  float position_rate_ = 0.0F;   // eye/s, low-passed: how fast the offset is moving
  std::uint64_t last_view_us_ = 0;
  bool in_dark_ = false;  // most of the bar is dark right now
  float dark_start_s_ = 0.0F;
  float corner_side_ = 1.0F;
  float corner_start_psi_ = 0.0F;
  std::size_t corner_stage_ = 0;
  std::size_t bypass_stage_ = 0;
  float bypass_start_psi_ = 0.0F;  // the line's heading before the detour
  bool braking_ = false;           // stopping the line-speed momentum before the turn
};

}  // namespace lf
