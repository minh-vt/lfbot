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

void test_speed_loop_reaches_target_both_ways() {
  for (const float target : {3000.0F, -3000.0F}) {
    Plant plant;
    lf::Motion motion;
    lf::Drive drive;
    std::uint64_t now_us = 0;
    for (int tick = 0; tick < 1000; ++tick) {  // 2 s: the floor-loaded motor has tau of 0.6 s
      now_us += Bench::kTickUs;
      const lf::Measurements m = plant.measure(now_us);
      motion.update(m);
      lf::Outputs outputs;
      outputs.motor = drive.step(motion, target, 0.0F, m.battery_v);
      require(std::abs(outputs.motor.duty[0]) <= lf::kMaxDuty &&
                std::abs(outputs.motor.duty[1]) <= lf::kMaxDuty,
              "duty never exceeds the limit");
      // The plant takes raw duty, so apply the wiring signs the way Robot does.
      for (std::size_t wheel = 0; wheel < lf::kWheelCount; ++wheel) {
        if (lf::kInvertWheel[wheel]) {
          outputs.motor.duty[wheel] = -outputs.motor.duty[wheel];
        }
      }
      plant.advance(outputs, lf::kControlDt);
    }
    require(std::abs(motion.wheel_cps(0) - target) < 0.05F * std::abs(target) &&
              std::abs(motion.wheel_cps(1) - target) < 0.05F * std::abs(target),
            "each wheel settles within 5% of target in two seconds, whatever its dead zone");
  }
}

void test_no_target_means_no_duty() {
  const lf::Motion motion;
  lf::Drive drive;
  lf::Sample<float> battery;
  battery.status = lf::SampleStatus::kValid;
  battery.value = 11.5F;
  const lf::MotorOutput output = drive.step(motion, 0.0F, 0.0F, battery);
  require(output.duty[0] == 0.0F && output.duty[1] == 0.0F, "zero target gives zero duty");
}

void test_unreachable_target_does_not_wind_up() {
  Plant plant;
  lf::Motion motion;
  lf::Drive drive;
  std::uint64_t now_us = 0;
  const auto run = [&](float target, int ticks) {
    for (int tick = 0; tick < ticks; ++tick) {
      now_us += Bench::kTickUs;
      const lf::Measurements m = plant.measure(now_us);
      motion.update(m);
      lf::Outputs outputs;
      outputs.motor = drive.step(motion, target, 0.0F, m.battery_v);
      for (std::size_t wheel = 0; wheel < lf::kWheelCount; ++wheel) {
        if (lf::kInvertWheel[wheel]) {
          outputs.motor.duty[wheel] = -outputs.motor.duty[wheel];
        }
      }
      plant.advance(outputs, lf::kControlDt);
    }
  };
  run(15000.0F, 1000);  // 2 s asking for more than the motor can give
  run(9000.0F, 1000);   // then a reachable target
  require(std::abs(motion.wheel_cps(0) - 9000.0F) < 450.0F,
          "settles on a reachable target within 2 s of saturating");
}

}  // namespace

void run_motion() {
  test_wheels_report_forward_as_positive();
  test_gyro_integrates_a_turn_and_bias_is_removed();
  test_a_shaken_robot_gives_no_bias_and_a_bump_is_waited_out();
  test_speed_loop_reaches_target_both_ways();
  test_no_target_means_no_duty();
  test_unreachable_target_does_not_wind_up();
  std::puts("motion: PASS (signs, odometry, gyro bias, speed loop)");
}
