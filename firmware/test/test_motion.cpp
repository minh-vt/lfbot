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

void test_direct_duty_applies_forward_and_turn() {
  Plant plant;
  lf::Motion motion;
  lf::Drive drive;
  std::uint64_t now_us = 0;
  const lf::Measurements m = plant.measure(now_us);
  motion.update(m);
  const lf::Sample<float>& battery = m.battery_v;
  const lf::MotorOutput straight = drive.step({0.5F, 0.0F}, battery, motion);
  require(straight.duty[0] > 0.4F && straight.duty[1] > 0.4F, "forward drives both wheels");
  const lf::MotorOutput left = drive.step({0.0F, 0.5F}, battery, motion);
  require(left.duty[0] < left.duty[1], "a positive turn (left) drives the right wheel harder");
  const lf::MotorOutput right = drive.step({0.0F, -0.5F}, battery, motion);
  require(right.duty[0] > right.duty[1], "a negative turn (right) drives the left wheel harder");
  const lf::MotorOutput clamped = drive.step({1.0F, 1.0F}, battery, motion);
  require(std::abs(clamped.duty[0]) <= lf::kMaxDuty && std::abs(clamped.duty[1]) <= lf::kMaxDuty,
          "duty never exceeds the limit");
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
  test_a_commanded_robot_that_does_not_move_is_stalled();
  std::puts("motion: PASS (signs, odometry, gyro bias, speed loop)");
}
