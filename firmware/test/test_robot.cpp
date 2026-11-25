#include <cmath>
#include <cstdio>

#include "check.hpp"
#include "config.hpp"
#include "plant.hpp"
#include "suites.hpp"

namespace {

// A long straight line, so a race is still on when a test interferes with it.
void lay_line(Bench& bench) {
  bench.plant.course = {{0.0F, 0.0F, 6000.0F, 0.0F}};
}

void racing(Bench& bench) {
  lay_line(bench);
  require(bench.start_race(), "calibrated and racing");
  bench.run_ms(1000);
}

void hold_button(Bench& bench, int ticks) {
  for (int tick = 0; tick < ticks; ++tick) {
    bench.tick(true);
  }
}

void test_button_stops_in_the_same_tick() {
  Bench bench;
  racing(bench);
  for (int tick = 0; tick < 40 && bench.phase() == lf::Phase::kRacing; ++tick) {
    bench.tick(true);
  }
  const lf::Snapshot snapshot = bench.robot.snapshot();
  require(snapshot.phase == lf::Phase::kStopped && snapshot.reason == lf::StopReason::kButton,
          "the button stops a running robot");
  require(bench.outputs.motor.mode == lf::MotorMode::kBrake, "and brakes it");
  require(bench.outputs.led == lf::Colour::kYellow, "and shows yellow");
}

void test_button_stops_while_settling() {
  Bench bench;
  lay_line(bench);
  bench.run_ms(100);
  hold_button(bench, 20);
  bench.run_ms(40);  // released past the debounce time
  require(bench.phase() == lf::Phase::kSettling, "settling after the press");
  hold_button(bench, 20);
  require(bench.phase() == lf::Phase::kStopped, "a second press stops a settling robot");
}

void test_a_bounce_is_not_a_press() {
  Bench bench;
  racing(bench);
  hold_button(bench, 6);  // 12 ms of contact, below the debounce time
  bench.run_ms(100);
  require(bench.phase() == lf::Phase::kRacing, "a 12 ms blip does not stop the run");
}

void test_stale_gyro_stops() {
  Bench bench;
  racing(bench);
  bench.plant.gyro_alive = false;
  bench.run_ms(100);
  const lf::Snapshot snapshot = bench.robot.snapshot();
  require(snapshot.phase == lf::Phase::kStopped && snapshot.reason == lf::StopReason::kGyroStale,
          "a gyro that stops updating stops the robot");
  require(bench.outputs.motor.mode != lf::MotorMode::kDrive, "and the wheels are no longer driven");
}

void test_wrong_gyro_sign_is_caught() {
  Bench bench;
  lay_line(bench);
  bench.plant.gyro_sign = -lf::kGyroSign;
  bench.run_ms(100);
  bench.press();
  bench.run_ms(10000);
  require(bench.robot.snapshot().reason == lf::StopReason::kGyroDisagrees,
          "a wrong gyro sign stops the scan instead of spinning away");
}

void test_jammed_wheel_stops() {
  Bench bench;
  racing(bench);
  bench.plant.jammed = true;
  bench.run_ms(3000);
  require(bench.robot.snapshot().reason == lf::StopReason::kStall, "a jammed wheel is detected");
}

void test_a_robot_being_moved_does_not_start() {
  Bench bench;
  lay_line(bench);
  bench.run_ms(100);
  hold_button(bench, 40);
  for (int tick = 0; tick < 1000; ++tick) {  // someone keeps carrying it for 2 s
    bench.plant.speed = {800.0F, 800.0F};
    bench.tick();
  }
  require(bench.phase() == lf::Phase::kSettling, "still settling while it moves");
  for (int tick = 0; tick < 1000; ++tick) {
    bench.plant.speed = {800.0F, 800.0F};
    bench.tick();
  }
  require(bench.robot.snapshot().reason == lf::StopReason::kNotStill,
          "a robot that never keeps still does not start");
}

void test_the_press_that_starts_it_does_not_prevent_it() {
  Bench bench;
  lay_line(bench);
  bench.run_ms(100);
  hold_button(bench, 40);
  for (int tick = 0; tick < 20; ++tick) {  // the finger nudging the robot
    bench.plant.speed = {tick % 2 == 0 ? 300.0F : 0.0F, 0.0F};
    bench.tick();
  }
  bench.plant.speed = {0.0F, 0.0F};
  require(bench.run_until(lf::Phase::kReady, 30.0F), "it calibrates once it is still again");
}

void test_low_battery_blocks_start_and_stops_a_run() {
  Bench low;
  lay_line(low);
  low.plant.battery_v = 9.0F;
  low.run_ms(100);
  low.press();
  require(low.phase() == lf::Phase::kIdle, "a low battery refuses to start");
  require(low.outputs.led == lf::Colour::kOrange, "and shows orange");

  Bench sagging;
  racing(sagging);
  sagging.plant.battery_v = 9.0F;
  sagging.run_ms(50);
  require(sagging.robot.snapshot().reason == lf::StopReason::kBattery,
          "a sagging battery stops a run");
}

void test_nothing_moves_without_a_press() {
  Bench bench;
  lay_line(bench);
  bench.run_ms(2000);
  require(bench.outputs.motor.mode != lf::MotorMode::kDrive, "an idle robot is never driven");
  require(bench.outputs.led == lf::Colour::kBlue, "and shows blue when ready");
}

void test_log_records_the_stop_reason() {
  Bench bench;
  racing(bench);
  bench.plant.gyro_alive = false;
  bench.run_ms(100);
  const lf::LogRing& log = bench.robot.log();
  require(log.size() > 0, "a run leaves a log");
  require(log.at(log.size() - 1).reason == lf::StopReason::kGyroStale,
          "the last record carries the reason");
}

void test_an_old_tof_reading_is_logged_as_missing() {
  Bench fresh;
  racing(fresh);
  const lf::LogRing& fresh_log = fresh.robot.log();
  require(fresh_log.at(fresh_log.size() - 1).tof_mm == 500, "a fresh reading is logged");

  Bench stale;
  stale.plant.tof_age_us = 1000000;
  racing(stale);
  const lf::LogRing& stale_log = stale.robot.log();
  require(stale_log.at(stale_log.size() - 1).tof_mm == -1,
          "a reading a second old is logged as missing");
}

}  // namespace

void run_robot() {
  test_button_stops_in_the_same_tick();
  test_button_stops_while_settling();
  test_a_bounce_is_not_a_press();
  test_stale_gyro_stops();
  test_wrong_gyro_sign_is_caught();
  test_jammed_wheel_stops();
  test_a_robot_being_moved_does_not_start();
  test_the_press_that_starts_it_does_not_prevent_it();
  test_low_battery_blocks_start_and_stops_a_run();
  test_nothing_moves_without_a_press();
  test_log_records_the_stop_reason();
  test_an_old_tof_reading_is_logged_as_missing();
  std::puts("robot: PASS (stop paths, safety stops, log)");
}
