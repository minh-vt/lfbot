#include <cmath>
#include <cstdio>

#include "check.hpp"
#include "config.hpp"
#include "plant.hpp"
#include "suites.hpp"

namespace {

lf::Command go(float mm) {
  return {lf::Command::Kind::kGo, mm};
}
lf::Command turn(float degrees) {
  return {lf::Command::Kind::kTurn, degrees};
}

// Powered up and still for long enough that the sensors have reported.
void warm_up(Bench& bench) {
  bench.run_ms(100);
}

void test_go_stops_at_the_distance() {
  Bench bench;
  warm_up(bench);
  bench.tick(false, go(500.0F));
  bench.run_until_stopped(10000);
  const lf::Snapshot snapshot = bench.robot.snapshot();
  require(snapshot.phase == lf::Phase::kIdle && snapshot.reason == lf::StopReason::kDone,
          "a finished go ends idle with reason done");
  require(std::abs(snapshot.s_mm - 500.0F) < 15.0F, "go stops within 15 mm of the distance");
  require(std::abs(snapshot.psi_deg) < 3.0F, "go keeps its heading");
}

void test_go_backwards() {
  Bench bench;
  warm_up(bench);
  bench.tick(false, go(-300.0F));
  bench.run_until_stopped(10000);
  require(std::abs(bench.robot.snapshot().s_mm + 300.0F) < 15.0F, "a negative go reverses");
}

void test_turn_reaches_the_angle_both_ways() {
  for (const float degrees : {90.0F, -90.0F}) {
    Bench bench;
    warm_up(bench);
    bench.tick(false, turn(degrees));
    bench.run_until_stopped(10000);
    const lf::Snapshot snapshot = bench.robot.snapshot();
    require(snapshot.reason == lf::StopReason::kDone, "a turn finishes");
    require(std::abs(snapshot.psi_deg - degrees) < 4.0F, "turn lands within 4 degrees");
  }
}

void test_button_stops_in_the_same_tick() {
  Bench bench;
  warm_up(bench);
  bench.tick(false, go(2000.0F));
  bench.run_ms(1500);
  require(bench.robot.snapshot().phase == lf::Phase::kRunning, "running before the press");
  // Hold the button past the debounce time.
  for (int tick = 0; tick < 40 && bench.robot.snapshot().phase == lf::Phase::kRunning; ++tick) {
    bench.tick(true);
  }
  const lf::Snapshot snapshot = bench.robot.snapshot();
  require(snapshot.phase == lf::Phase::kStopped && snapshot.reason == lf::StopReason::kButton,
          "the button stops a running robot");
  require(bench.outputs.motor.mode == lf::MotorMode::kBrake, "and brakes it");
  require(bench.outputs.led.colour == lf::Colour::kYellow, "and shows yellow");
}

void test_button_stops_while_settling() {
  Bench bench;
  warm_up(bench);
  bench.tick(false, go(500.0F));
  require(bench.robot.snapshot().phase == lf::Phase::kSettling, "settling after the command");
  for (int tick = 0; tick < 40; ++tick) {
    bench.tick(true);
  }
  require(bench.robot.snapshot().phase == lf::Phase::kStopped,
          "the button also stops a settling robot");
}

void test_a_bounce_is_not_a_press() {
  Bench bench;
  warm_up(bench);
  bench.tick(false, go(2000.0F));
  bench.run_ms(1000);
  for (int tick = 0; tick < 6; ++tick) {  // 12 ms of contact, below the debounce time
    bench.tick(true);
  }
  bench.run_ms(100);
  require(bench.robot.snapshot().phase == lf::Phase::kRunning,
          "a 12 ms blip does not stop the run");
}

void test_stale_gyro_stops() {
  Bench bench;
  warm_up(bench);
  bench.tick(false, go(2000.0F));
  bench.run_ms(1000);
  bench.plant.gyro_alive = false;
  bench.run_ms(100);
  const lf::Snapshot snapshot = bench.robot.snapshot();
  require(snapshot.phase == lf::Phase::kStopped && snapshot.reason == lf::StopReason::kGyroStale,
          "a gyro that stops updating stops the robot");
  require(bench.outputs.motor.mode != lf::MotorMode::kDrive, "and the wheels are no longer driven");
}

void test_wrong_gyro_sign_is_caught() {
  Bench bench;
  bench.plant.gyro_sign = -lf::kGyroSign;
  warm_up(bench);
  bench.tick(false, turn(90.0F));
  bench.run_until_stopped(10000);
  require(bench.robot.snapshot().reason == lf::StopReason::kGyroDisagrees,
          "a wrong gyro sign stops the turn instead of spinning away");
}

void test_jammed_wheel_stops() {
  Bench bench;
  warm_up(bench);
  bench.tick(false, go(2000.0F));
  bench.run_ms(700);
  bench.plant.jammed = true;
  bench.run_until_stopped(3000);
  require(bench.robot.snapshot().reason == lf::StopReason::kStall, "a jammed wheel is detected");
}

void test_a_robot_being_moved_does_not_start() {
  Bench bench;
  warm_up(bench);
  bench.tick(false, go(500.0F));
  for (int tick = 0; tick < 1000; ++tick) {  // someone keeps carrying it for 2 s
    bench.plant.speed = {800.0F, 800.0F};
    bench.tick();
  }
  require(bench.robot.snapshot().phase == lf::Phase::kSettling, "still settling while it moves");
  for (int tick = 0; tick < 1000; ++tick) {
    bench.plant.speed = {800.0F, 800.0F};
    bench.tick();
  }
  require(bench.robot.snapshot().reason == lf::StopReason::kNotStill,
          "a robot that never keeps still does not start");
}

void test_the_press_that_starts_it_does_not_prevent_it() {
  Bench bench;
  warm_up(bench);
  bench.tick(false, go(300.0F));
  bench.plant.speed = {0.0F, 0.0F};
  for (int tick = 0; tick < 20; ++tick) {  // the finger nudging the robot
    bench.plant.speed = {tick % 2 == 0 ? 300.0F : 0.0F, 0.0F};
    bench.tick();
  }
  bench.plant.speed = {0.0F, 0.0F};
  bench.run_until_stopped(10000);
  require(bench.robot.snapshot().reason == lf::StopReason::kDone,
          "it starts once it is still again");
}

void test_low_battery_blocks_start_and_stops_a_run() {
  Bench low;
  low.plant.battery_v = 9.0F;
  warm_up(low);
  low.tick(false, go(500.0F));
  require(low.robot.snapshot().phase == lf::Phase::kIdle, "a low battery refuses to start");
  require(low.outputs.led.colour == lf::Colour::kOrange, "and shows orange");

  Bench running;
  warm_up(running);
  running.tick(false, go(2000.0F));
  running.run_ms(1000);
  running.plant.battery_v = 9.0F;
  running.run_ms(50);
  require(running.robot.snapshot().reason == lf::StopReason::kBattery,
          "a sagging battery stops a run");
}

void test_nothing_moves_without_a_command() {
  Bench bench;
  bench.run_ms(2000);
  require(bench.outputs.motor.mode != lf::MotorMode::kDrive, "an idle robot is never driven");
  require(bench.outputs.led.colour == lf::Colour::kBlue, "and shows blue when ready");
}

void test_log_records_the_stop_reason() {
  Bench bench;
  warm_up(bench);
  bench.tick(false, go(2000.0F));
  bench.run_ms(1000);
  bench.plant.gyro_alive = false;
  bench.run_ms(100);
  const lf::LogRing& log = bench.robot.log();
  require(log.size() > 0, "a run leaves a log");
  require(log.at(log.size() - 1).reason == lf::StopReason::kGyroStale,
          "the last record carries the reason");
}

void test_one_press_runs_the_bench_routine() {
  Bench bench;
  warm_up(bench);
  for (int tick = 0; tick < 40; ++tick) {
    bench.tick(true);
  }
  require(bench.robot.snapshot().routine_active, "a press starts the routine");
  bool saw_turn = false;
  for (int tick = 0; tick < 20000 && bench.robot.snapshot().routine_active; ++tick) {
    bench.tick();
    saw_turn = saw_turn || std::abs(bench.robot.snapshot().psi_deg) > 30.0F;
  }
  const lf::Snapshot snapshot = bench.robot.snapshot();
  require(!snapshot.routine_active && snapshot.reason == lf::StopReason::kDone,
          "the routine runs every step and ends done");
  require(saw_turn, "and it included a turn");
  require(std::abs(snapshot.s_mm + 300.0F) < 15.0F, "the last step reversed 300 mm");
}

void test_a_press_cancels_the_routine() {
  Bench bench;
  warm_up(bench);
  for (int tick = 0; tick < 40; ++tick) {
    bench.tick(true);
  }
  bench.run_ms(1000);
  for (int tick = 0; tick < 40; ++tick) {
    bench.tick(true);
  }
  require(!bench.robot.snapshot().routine_active, "a press ends the routine");
  bench.run_ms(3000);
  require(bench.robot.snapshot().phase == lf::Phase::kStopped, "and no later step starts");
}

void test_an_old_tof_reading_is_logged_as_missing() {
  Bench fresh;
  warm_up(fresh);
  fresh.tick(false, go(300.0F));
  fresh.run_ms(300);
  const lf::LogRing& fresh_log = fresh.robot.log();
  require(fresh_log.at(fresh_log.size() - 1).tof_mm == 500, "a fresh reading is logged");

  Bench stale;
  stale.plant.tof_age_us = 1000000;
  warm_up(stale);
  stale.tick(false, go(300.0F));
  stale.run_ms(300);
  const lf::LogRing& stale_log = stale.robot.log();
  require(stale_log.at(stale_log.size() - 1).tof_mm == -1,
          "a reading a second old is logged as missing");
}

void test_a_finished_move_brakes_briefly_then_releases() {
  Bench bench;
  warm_up(bench);
  bench.tick(false, turn(45.0F));
  bool braked = false;
  for (int tick = 0; tick < 10000 && bench.robot.snapshot().reason != lf::StopReason::kDone;
       ++tick) {
    bench.tick();
  }
  for (int tick = 0; tick < 30; ++tick) {
    bench.tick();
    braked = braked || bench.outputs.motor.mode == lf::MotorMode::kBrake;
  }
  require(braked, "the wheels are braked when a move ends");
  bench.run_ms(500);
  require(bench.outputs.motor.mode == lf::MotorMode::kCoast, "and released afterwards");
}

}  // namespace

void run_robot() {
  test_go_stops_at_the_distance();
  test_go_backwards();
  test_turn_reaches_the_angle_both_ways();
  test_button_stops_in_the_same_tick();
  test_button_stops_while_settling();
  test_a_bounce_is_not_a_press();
  test_stale_gyro_stops();
  test_wrong_gyro_sign_is_caught();
  test_jammed_wheel_stops();
  test_a_robot_being_moved_does_not_start();
  test_the_press_that_starts_it_does_not_prevent_it();
  test_low_battery_blocks_start_and_stops_a_run();
  test_nothing_moves_without_a_command();
  test_log_records_the_stop_reason();
  test_an_old_tof_reading_is_logged_as_missing();
  test_a_finished_move_brakes_briefly_then_releases();
  test_one_press_runs_the_bench_routine();
  test_a_press_cancels_the_routine();
  std::puts("robot: PASS (go, turn, stop paths, safety stops, log)");
}
