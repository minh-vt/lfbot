#include <cassert>
#include <iomanip>
#include <iostream>

#include "race_arbiter.hpp"
#include "sim_engine.hpp"

namespace {

void test_sim_run1_map_learning(lf::sim::SimEngine& engine, float& out_run1_time_s) {
  std::cout << "[SIM TEST] Running Run 1: Map Scanning & C2 Trajectory Generation...\n";

  const std::vector<lf::sim::Obstacle> obstacles = {
    {{1100.0f, 300.0f}, 105.0f, 60.0f, 0.0f},
  };
  engine.set_obstacles(obstacles);

  const auto report = engine.run_lap(lf::RobotMode::kRun1Map, 25.0f);
  if (!report.success) {
    engine.dump_post_mortem(std::cerr, report);
  }
  assert(report.success);
  assert(report.verdict.status == lf::sim::RaceStatus::kCompleted);
  assert(report.verdict.sectors_cleared == report.verdict.total_sectors);
  assert(report.verdict.stopped_in_finish_box);
  assert(report.verdict.continuous_off_line_s < 5.0f);
  assert(report.lap_time_s >= 8.0f && report.lap_time_s <= 22.0f);
  assert(report.waypoints_count >= 100);
  assert(engine.get_state().has_run1_data);
  assert(!engine.get_state().planned_trajectory.empty());
  assert(engine.get_robot().get_follower().has_trajectory());

  out_run1_time_s = report.lap_time_s;
  std::cout << "  -> Run 1 PASSED: Time = " << std::fixed << std::setprecision(2)
            << report.lap_time_s << "s, Distance = " << report.total_distance_m
            << "m, Sectors = " << report.verdict.sectors_cleared << "/"
            << report.verdict.total_sectors
            << ", Finish Box Halt = YES, Avg Speed = " << report.avg_speed_mps
            << " m/s, Min Obs Dist = " << report.min_obstacle_clearance_mm << " mm\n";
}

void test_sim_run2_c2_sprint_evasion(lf::sim::SimEngine& engine, float run1_time_s) {
  std::cout << "[SIM TEST] Running Run 2: C2 Racing Line Sprint with Obstacle Avoidance...\n";

  const auto report = engine.run_lap(lf::RobotMode::kRun2Sprint, 22.0f);
  if (!report.success) {
    engine.dump_post_mortem(std::cerr, report);
  }
  assert(report.success);
  assert(report.verdict.status == lf::sim::RaceStatus::kCompleted);
  assert(report.verdict.sectors_cleared == report.verdict.total_sectors);
  assert(report.verdict.stopped_in_finish_box);
  assert(report.verdict.continuous_off_line_s < 5.0f);
  assert(report.total_distance_m >= 5.5f);  // Poka-Yoke: MUST complete full lap!
  assert(report.waypoints_count >= 500);
  assert(report.lap_time_s >= 2.0f && report.lap_time_s <= 7.0f);

  const float delta_s = report.lap_time_s - run1_time_s;
  const float pct = (delta_s / run1_time_s) * 100.0f;

  assert(report.lap_time_s < run1_time_s);  // Sprint MUST be faster than mapping run!

  constexpr float kRequiredClearanceMm = 150.0f;
  assert(report.min_obstacle_clearance_mm >= kRequiredClearanceMm);

  std::cout << "  -> Run 2 PASSED: Time = " << std::fixed << std::setprecision(2)
            << report.lap_time_s << "s [Delta: " << delta_s << "s (" << pct
            << "%)], Sectors = " << report.verdict.sectors_cleared << "/"
            << report.verdict.total_sectors
            << ", Min Clearance = " << report.min_obstacle_clearance_mm
            << " mm, Top Speed = " << report.top_speed_mps
            << " m/s, Max CTE = " << report.max_cross_track_error_mm << " mm\n";
}

void test_sim_failure_post_mortem_diagnostics(lf::sim::SimEngine& engine) {
  std::cout << "[SIM TEST] Verifying Crash Post-Mortem Diagnostics Harness...\n";

  // Intentionally place an obstacle directly in front of start pose at (580, 1300)
  const std::vector<lf::sim::Obstacle> impassable_obs = {
    {{580.0f, 1300.0f}, 150.0f, 150.0f, 0.0f},
  };
  engine.set_obstacles(impassable_obs);

  const auto report = engine.run_lap(lf::RobotMode::kRun1Map, 5.0f);
  // Must detect collision and fail gracefully
  assert(!report.success);
  assert(report.verdict.status == lf::sim::RaceStatus::kDisqualified);
  assert(report.verdict.infraction == lf::sim::InfractionCode::kObstacleCollision);
  assert(report.crash_record.timestamp_s > 0.0f);
  assert(report.crash_record.x_mm > 500.0f && report.crash_record.x_mm <= 650.0f);
  assert(report.min_obstacle_clearance_mm < 130.0f);

  // Dump JSON crash log to verify serializer
  const bool json_ok = engine.dump_crash_json(".sim_crash_test.json", report);
  assert(json_ok);

  std::cout << "  -> Crash Post-Mortem Reporter PASSED: Correctly caught collision at t="
            << report.crash_record.timestamp_s << "s, X=" << report.crash_record.x_mm
            << "mm, Dist=" << report.min_obstacle_clearance_mm << "mm\n";
}

void test_arbiter_out_of_bounds() {
  std::cout << "[SIM TEST] Verifying Arbiter Invariant: Out-of-Bounds Disqualification...\n";
  lf::sim::RaceArbiter arbiter;
  arbiter.reset();

  lf::sim::ArbiterGroundTruth gt{};
  gt.x_mm = 500.0f;
  gt.y_mm = 1300.0f;
  gt.line_detected = true;
  arbiter.step(gt, 0.01f);
  assert(arbiter.get_verdict().status == lf::sim::RaceStatus::kInProgress);

  // Breach arena boundary: X > 2450 mm
  gt.x_mm = 2500.0f;
  arbiter.step(gt, 0.01f);
  assert(arbiter.get_verdict().status == lf::sim::RaceStatus::kDisqualified);
  assert(arbiter.get_verdict().infraction == lf::sim::InfractionCode::kOutOfBounds);
  std::cout << "  -> Out-of-Bounds PASSED: Disqualified on arena wall breach (X > 2450mm)\n";
}

void test_arbiter_wrong_way() {
  std::cout << "[SIM TEST] Verifying Arbiter Invariant: Reverse Direction Crossing...\n";
  lf::sim::RaceArbiter arbiter;
  arbiter.reset();

  // Gate 1: X=900, normal (1, 0)
  // Step 1: Initialize on east side of gate
  lf::sim::ArbiterGroundTruth gt{};
  gt.x_mm = 950.0f;
  gt.y_mm = 1300.0f;
  gt.line_detected = true;
  arbiter.step(gt, 0.01f);

  // Step 2: Traverse westward (reverse direction) across gate line
  gt.x_mm = 850.0f;
  arbiter.step(gt, 0.01f);

  assert(arbiter.get_verdict().status == lf::sim::RaceStatus::kDisqualified);
  assert(arbiter.get_verdict().infraction == lf::sim::InfractionCode::kWrongWay);
  std::cout << "  -> Wrong-Way PASSED: Disqualified on reverse gate crossing\n";
}

void test_arbiter_line_lost_5s() {
  std::cout << "[SIM TEST] Verifying Arbiter Invariant: Rule 5.5 (5.0s Off-Line Limit)...\n";
  lf::sim::RaceArbiter arbiter;
  arbiter.reset();

  lf::sim::ArbiterGroundTruth gt{};
  gt.x_mm = 500.0f;
  gt.y_mm = 1300.0f;
  gt.line_detected = true;
  arbiter.step(gt, 0.01f);
  assert(arbiter.get_verdict().status == lf::sim::RaceStatus::kInProgress);

  // Lose line for 4.9s (within allowable tolerance)
  gt.line_detected = false;
  for (int i = 0; i < 490; ++i) {
    arbiter.step(gt, 0.01f);
  }
  assert(arbiter.get_verdict().status == lf::sim::RaceStatus::kInProgress);
  assert(arbiter.get_verdict().continuous_off_line_s >= 4.89f);

  // Exceed 5.0s threshold (further 0.2s = 5.1s)
  for (int i = 0; i < 20; ++i) {
    arbiter.step(gt, 0.01f);
  }
  assert(arbiter.get_verdict().status == lf::sim::RaceStatus::kDisqualified);
  assert(arbiter.get_verdict().infraction == lf::sim::InfractionCode::kLineLostTimeout);
  std::cout << "  -> Line-Lost Limit PASSED: Disqualified precisely after 5.0s off-line\n";
}

void test_arbiter_gate_skipped() {
  std::cout << "[SIM TEST] Verifying Arbiter Invariant: Sector Chronology & Gate Skipping...\n";
  lf::sim::RaceArbiter arbiter;
  arbiter.reset();

  // 3 sequential parallel gates
  const std::vector<lf::sim::SectorGate> custom_gates = {
    {100.0f, 0.0f, 100.0f, 100.0f, 1.0f, 0.0f},
    {200.0f, 0.0f, 200.0f, 100.0f, 1.0f, 0.0f},
    {300.0f, 0.0f, 300.0f, 100.0f, 1.0f, 0.0f},
  };
  arbiter.set_sector_gates(custom_gates);

  // 1. Cross Gate 0 legally (50 -> 150)
  lf::sim::ArbiterGroundTruth gt{};
  gt.x_mm = 50.0f;
  gt.y_mm = 50.0f;
  gt.line_detected = true;
  arbiter.step(gt, 0.01f);

  gt.x_mm = 150.0f;
  arbiter.step(gt, 0.01f);
  assert(arbiter.get_verdict().sectors_cleared == 1);

  // 2. Bypass Gate 1 by moving above it (Y=200, outside gate line Y in [0, 100])
  gt.x_mm = 250.0f;
  gt.y_mm = 200.0f;
  arbiter.step(gt, 0.01f);

  gt.y_mm = 50.0f;
  arbiter.step(gt, 0.01f);

  // 3. Cross Gate 2 (X=300) in forward direction without clearing Gate 1
  gt.x_mm = 350.0f;
  arbiter.step(gt, 0.01f);

  assert(arbiter.get_verdict().status == lf::sim::RaceStatus::kDisqualified);
  assert(arbiter.get_verdict().infraction == lf::sim::InfractionCode::kGateSkipped);
  std::cout << "  -> Gate Skipping PASSED: Disqualified when skipping Sector Gate\n";
}

void test_arbiter_finish_box_validation() {
  std::cout << "[SIM TEST] Verifying Arbiter Invariant: Finish Box & Premature Finish Reject...\n";
  lf::sim::RaceArbiter arbiter;
  arbiter.reset();

  // Enter finish box at start without clearing any gates
  lf::sim::ArbiterGroundTruth gt{};
  gt.x_mm = 400.0f;
  gt.y_mm = 1300.0f;
  gt.v_mps = 0.0f;
  gt.active_brake = true;
  gt.line_detected = true;

  arbiter.step(gt, 0.01f);
  for (int i = 0; i < 30; ++i) {
    arbiter.step(gt, 0.01f);
  }
  // Cannot complete if sectors_cleared < total_sectors!
  assert(arbiter.get_verdict().status != lf::sim::RaceStatus::kCompleted);
  std::cout << "  -> Finish Box Invariant PASSED: Rejected premature halt without all sectors\n";
}

void test_arbiter_physics_anomaly() {
  std::cout << "[SIM TEST] Verifying Arbiter Invariant: Physics Anomaly Protection (< 2.0s)...\n";
  lf::sim::RaceArbiter arbiter;
  arbiter.reset();

  // Set 1 gate only
  const std::vector<lf::sim::SectorGate> one_gate = {
    {900.0f, 1100.0f, 900.0f, 1500.0f, 1.0f, 0.0f},
  };
  arbiter.set_sector_gates(one_gate);

  // Step 1: Legal gate crossing
  lf::sim::ArbiterGroundTruth gt{};
  gt.x_mm = 850.0f;
  gt.y_mm = 1300.0f;
  gt.line_detected = true;
  arbiter.step(gt, 0.01f);

  gt.x_mm = 950.0f;
  arbiter.step(gt, 0.01f);
  assert(arbiter.get_verdict().sectors_cleared == 1);

  // Step 2: Move outside gate line (Y=200, outside Y in [1100, 1500]) towards finish box
  gt.x_mm = 950.0f;
  gt.y_mm = 200.0f;
  arbiter.step(gt, 0.01f);

  gt.x_mm = 400.0f;
  gt.y_mm = 200.0f;
  arbiter.step(gt, 0.01f);

  // Enter finish box at (400, 1300) with controlled halt at t = 0.05s (< 2.0s physics anomaly)
  gt.x_mm = 400.0f;
  gt.y_mm = 1300.0f;
  gt.v_mps = 0.0f;
  gt.active_brake = true;
  arbiter.step(gt, 0.01f);

  assert(arbiter.get_verdict().status == lf::sim::RaceStatus::kDisqualified);
  assert(arbiter.get_verdict().infraction == lf::sim::InfractionCode::kPhysicsAnomaly);
  std::cout << "  -> Physics Anomaly PASSED: Disqualified impossible lap duration (< 2.0s)\n";
}

}  // namespace

int main() {
  std::cout << "================================================================================\n";
  std::cout << "        LF-BOT 2026: HEADLESS SIMULATION SMOKE-TEST (VIRTUAL CLOCK SSoT)        \n";
  std::cout << "================================================================================\n";

  lf::sim::SimEngine engine;
  const bool loaded = engine.load_track("assets/track.png");
  if (!loaded) {
    std::cerr << "[ERROR] Could not load assets/track.png! Run from firmware directory.\n";
    return 1;
  }

  // 1. End-to-end full race simulations
  float run1_time_s = 0.0f;
  test_sim_run1_map_learning(engine, run1_time_s);
  test_sim_run2_c2_sprint_evasion(engine, run1_time_s);
  test_sim_failure_post_mortem_diagnostics(engine);

  // 2. Adversarial Arbiter Invariant Unit Tests
  test_arbiter_out_of_bounds();
  test_arbiter_wrong_way();
  test_arbiter_line_lost_5s();
  test_arbiter_gate_skipped();
  test_arbiter_finish_box_validation();
  test_arbiter_physics_anomaly();

  std::cout << "================================================================================\n";
  std::cout << "               ALL HEADLESS SIMULATION INVARIANTS PASSED (SUCCESS)              \n";
  std::cout << "================================================================================\n";

  return 0;
}
