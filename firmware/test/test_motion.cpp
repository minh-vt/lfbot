#include <algorithm>
#include <cmath>
#include <cstdio>

#include "check.hpp"
#include "config.hpp"
#include "drive.hpp"
#include "motion.hpp"
#include "plant.hpp"
#include "suites.hpp"

namespace {

void test_wheels_report_forward_as_positive() {
  Plant plant;
  lf::Motion motion;
  lf::Outputs outputs;
  outputs.motor.mode = lf::MotorMode::kDrive;
  // Raw duty that drives both wheels forward carries the wiring signs.
  outputs.motor.duty = {lf::kInvertWheel[0] ? -0.7F : 0.7F, lf::kInvertWheel[1] ? -0.7F : 0.7F};
  std::uint64_t now_us = 0;
  for (int tick = 0; tick < 300; ++tick) {
    now_us += Bench::kTickUs;
    motion.update(plant.measure(now_us));
    plant.advance(outputs, lf::kControlDt);
  }
  require(motion.wheel_cps(0) > 1000.0F && motion.wheel_cps(1) > 1000.0F,
          "both wheels read forward speed as positive");
  require(motion.s_counts() > 500.0F, "distance grows going forward");
  require(std::abs(motion.psi_rad()) < 0.05F, "equal wheels do not turn the heading");
}

// Feeds the motion estimator until it finds a still block, or gives up.
bool find_bias(Plant& plant, lf::Motion& motion, std::uint64_t& now_us, int max_ticks) {
  motion.begin_bias();
  for (int tick = 0; tick < max_ticks; ++tick) {
    now_us += Bench::kTickUs;
    motion.update(plant.measure(now_us));
    if (motion.update_bias(now_us)) {
      return true;
    }
  }
  return false;
}

void test_gyro_integrates_a_turn_and_bias_is_removed() {
  Plant plant;
  lf::Motion motion;
  std::uint64_t now_us = 0;
  require(find_bias(plant, motion, now_us, 500), "a still gyro gives a bias");
  require(std::abs(motion.bias_rad_s() - Plant::kGyroBiasRadS) < 1e-4F,
          "bias is the still reading");
  motion.zero();
  // Left wheel back, right wheel forward at 1000 counts/s each: a left turn.
  const float rate = 2000.0F / lf::kTrackCountsPerRad;
  plant.speed = {-1000.0F, 1000.0F};
  for (int tick = 0; tick < 500; ++tick) {  // 1 s
    now_us += Bench::kTickUs;
    motion.update(plant.measure(now_us));
  }
  require(std::abs(motion.psi_rad() - rate) < 0.02F, "heading integrates rate over time");
  require(motion.psi_rad() > 0.0F, "a left turn is positive");
}

// Position on the floor follows the heading: a quarter turn then straight goes sideways.
void test_odometry_tracks_position_on_the_floor() {
  Plant plant;
  lf::Motion motion;
  std::uint64_t now_us = 0;
  require(find_bias(plant, motion, now_us, 500), "still at the start");
  motion.zero();
  const auto run = [&](float left, float right, int ticks) {
    for (int tick = 0; tick < ticks; ++tick) {
      now_us += Bench::kTickUs;
      plant.speed = {left, right};
      for (std::size_t wheel = 0; wheel < lf::kWheelCount; ++wheel) {
        plant.counts[wheel] += static_cast<double>(plant.speed[wheel] * lf::kControlDt);
      }
      plant.yaw_rad +=
        static_cast<double>((right - left) / lf::kTrackCountsPerRad * lf::kControlDt);
      motion.update(plant.measure(now_us));
    }
  };
  run(3000.0F, 3000.0F, 500);  // straight ahead
  require(
    std::abs(motion.x_mm() - 3000.0F * lf::kMmPerCount) < 5.0F && std::abs(motion.y_mm()) < 2.0F,
    "straight ahead is along x");
  const float x_before = motion.x_mm();
  const int quarter = static_cast<int>(1.5707963F * lf::kTrackCountsPerRad / 2000.0F * 500.0F);
  run(-1000.0F, 1000.0F, quarter);  // a quarter turn left in place
  run(3000.0F, 3000.0F, 500);
  require(std::abs(motion.x_mm() - x_before) < 10.0F, "after a left quarter turn x stays");
  require(std::abs(motion.y_mm() - 3000.0F * lf::kMmPerCount) < 10.0F, "and y grows to the left");
}

void test_a_shaken_robot_gives_no_bias_and_a_bump_is_waited_out() {
  Plant shaken;
  lf::Motion motion;
  std::uint64_t now_us = 0;
  motion.begin_bias();
  bool found = false;
  for (int tick = 0; tick < 1500 && !found; ++tick) {  // 3 s of shaking
    now_us += Bench::kTickUs;
    shaken.speed = {0.0F, tick % 2 == 0 ? 0.0F : 400.0F};
    motion.update(shaken.measure(now_us));
    found = motion.update_bias(now_us);
  }
  require(!found, "a robot that keeps moving gives no bias");

  // A bump from pressing the button, then stillness: the next block is accepted.
  Plant bumped;
  lf::Motion second;
  now_us = 0;
  second.begin_bias();
  bool accepted = false;
  for (int tick = 0; tick < 1000 && !accepted; ++tick) {
    now_us += Bench::kTickUs;
    bumped.speed = {0.0F, tick < 60 && tick % 2 == 0 ? 400.0F : 0.0F};
    second.update(bumped.measure(now_us));
    accepted = second.update_bias(now_us);
  }
  require(accepted, "a short bump does not stop the robot starting once it is still");
}

// Steps the drive for `ticks` ticks with the same demand; returns the last output.
lf::MotorOutput hold(lf::Drive& drive, const lf::Target& target, int ticks) {
  Plant plant;
  lf::Motion motion;
  const lf::Measurements m = plant.measure(0);
  motion.update(m);
  lf::MotorOutput output;
  for (int tick = 0; tick < ticks; ++tick) {
    output = drive.step(target, m.battery_v, motion);
  }
  return output;
}

void test_direct_duty_applies_forward_and_turn() {
  lf::Drive drive;
  const lf::MotorOutput straight = hold(drive, {0.5F, 0.0F}, 500);
  require(straight.duty[0] > 0.4F && straight.duty[1] > 0.4F, "forward drives both wheels");
  require(std::abs(straight.duty[0] - straight.duty[1]) < 1e-4F, "and evenly");
  drive.reset();
  const lf::MotorOutput left = hold(drive, {0.0F, 0.5F}, 1);
  require(left.duty[0] < left.duty[1], "a positive turn (left) drives the right wheel harder");
  const lf::MotorOutput right = hold(drive, {0.0F, -0.5F}, 1);
  require(right.duty[0] > right.duty[1], "a negative turn (right) drives the left wheel harder");
  const lf::MotorOutput clamped = hold(drive, {1.0F, 1.0F}, 500);
  require(std::abs(clamped.duty[0]) <= lf::kMaxDuty && std::abs(clamped.duty[1]) <= lf::kMaxDuty,
          "duty never exceeds the limit");
}

// A saturated wheel must not eat the turn: forward gives way instead.
void test_steering_wins_when_a_wheel_saturates() {
  lf::Drive drive;
  const lf::MotorOutput output = hold(drive, {0.8F, 0.5F}, 500);
  const lf::MotorOutput turn_only = hold(drive, {0.0F, 0.5F}, 500);
  const float wanted = turn_only.duty[1] - turn_only.duty[0];
  require(std::abs((output.duty[1] - output.duty[0]) - wanted) < 1e-3F,
          "the wheel difference is the whole turn even at full forward");
  require(output.duty[0] + output.duty[1] > 0.5F, "and what forward is left still drives forward");
}

// The robot cannot be thrown from standstill to full duty in one tick.
void test_forward_rises_gradually_and_falls_quickly() {
  lf::Drive drive;
  const lf::MotorOutput first = hold(drive, {0.7F, 0.0F}, 1);
  require(first.duty[0] < 0.1F, "one tick after a start the push is still small");
  const lf::MotorOutput cruising = hold(drive, {0.7F, 0.0F}, 1000);
  require(cruising.duty[0] > 0.6F, "and it reaches the demand");
  const int rise_ticks = static_cast<int>(0.7F / lf::kForwardRisePerTick);
  const lf::MotorOutput slowed = hold(drive, {0.0F, 0.0F}, rise_ticks / 4);
  require(slowed.duty[0] < 0.05F, "slowing down is at least four times faster than speeding up");
}

void test_a_commanded_robot_that_does_not_move_is_stalled() {
  Plant plant;
  lf::Motion motion;
  lf::Drive drive;
  plant.jammed = true;
  std::uint64_t now_us = 0;
  for (int tick = 0; tick < 1000; ++tick) {
    now_us += Bench::kTickUs;
    const lf::Measurements m = plant.measure(now_us);
    motion.update(m);
    static_cast<void>(drive.step({0.6F, 0.0F}, m.battery_v, motion));
  }
  require(drive.stalled(), "a commanded robot with wheels that never turn is jammed");
}

}  // namespace

void run_motion() {
  test_wheels_report_forward_as_positive();
  test_gyro_integrates_a_turn_and_bias_is_removed();
  test_a_shaken_robot_gives_no_bias_and_a_bump_is_waited_out();
  test_direct_duty_applies_forward_and_turn();
  test_steering_wins_when_a_wheel_saturates();
  test_forward_rises_gradually_and_falls_quickly();
  test_odometry_tracks_position_on_the_floor();
  test_a_commanded_robot_that_does_not_move_is_stalled();
  std::puts("motion: PASS (signs, odometry, position, gyro bias, wheel mixing)");
}
