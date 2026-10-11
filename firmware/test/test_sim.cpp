#include <cstdio>

#include "drive/plan.hpp"
#include "drive/step.hpp"
#include "race_arbiter.hpp"
#include "track_asset.hpp"
#include "world_physics.hpp"

namespace {

constexpr const char* kManifestPath = "assets/track_vongloai.json";
constexpr float kDt = 0.002f;      // 500Hz
constexpr int kMaxSteps = 120000;  // 240s

}

int main() {
  using namespace lf;

  sim::TrackAsset track;
  if (!track.load(kManifestPath)) {
    (void)std::printf("test_sim: FAIL cannot load track\n");
    return 1;
  }

  sim::WorldPhysics world;
  world.reset(track.manifest());
  sim::RaceArbiter arbiter;
  arbiter.reset(track.manifest());

  const drive::Tuning cfg;
  drive::DriveState state;
  drive::RacePlan plan;
  drive::RunMode mode = drive::RunMode::kRun1;
  float run_elapsed = 0.0f;
  float run1_time = 0.0f;
  float run2_time = 0.0f;
  sim::Verdict run1_verdict;
  sim::Verdict run2_verdict;

  int steps = 0;
  while (steps++ < kMaxSteps && mode != drive::RunMode::kDone) {
    drive::Readings frame;
    world.sample(frame, kDt, track);
    if (steps % 2000 == 0) {
      (void)std::printf("  [s=%5d t=%.0fs] pos=(%.0f,%.0f) th=%3.0f err=%+.2f cert=%.2f\n", steps,
                        run_elapsed, world.x(), world.y(), world.theta() * (180.0f / sim::kPi),
                        frame.line_offset, frame.line_certainty);
    }
    const auto result = drive::step(state, frame, plan, cfg);
    state = result.state;
    if (mode == drive::RunMode::kRun1) {
      const float tick = 0.5f * (static_cast<float>(frame.enc_l) + static_cast<float>(frame.enc_r));
      drive::record(plan, tick, result.path_steer, frame.obstacle_side);
    }
    world.step(result.command, kDt);
    arbiter.observe(world.x(), world.y(), world.v(), frame.line_certainty > 0.5f, kDt);
    run_elapsed += kDt;

    if (mode == drive::RunMode::kRun1 && state.finish_latched) {
      run1_verdict = arbiter.verdict();
      run1_time = run_elapsed;
      drive::plan_speed(plan, cfg);
      world.reset(track.manifest());
      arbiter.reset(track.manifest());
      state = {};
      run_elapsed = 0.0f;
      mode = drive::RunMode::kRun2;
    } else if (mode == drive::RunMode::kRun2 && state.finish_latched) {
      run2_verdict = arbiter.verdict();
      run2_time = run_elapsed;
      mode = drive::RunMode::kDone;
    }
  }

  (void)std::printf("test_sim: steps=%d  Run1=%.2fs (%s)  Run2=%.2fs (%s)  plan=%zu ticks\n", steps,
                    run1_time, run1_verdict.completed ? "OK" : "DQ", run2_time,
                    run2_verdict.completed ? "OK" : "DQ", plan.planned_steer.size());

  bool pass = true;
  if (!run1_verdict.completed) {
    (void)std::printf("test_sim: FAIL Run1 (infraction=%d)\n",
                      static_cast<int>(run1_verdict.infraction));
    pass = false;
  }
  if (!run2_verdict.completed) {
    (void)std::printf("test_sim: FAIL Run2 (infraction=%d)\n",
                      static_cast<int>(run2_verdict.infraction));
    pass = false;
  }
  if (run1_verdict.completed && run2_verdict.completed && run2_time >= run1_time) {
    (void)std::printf("test_sim: FAIL Run2 (%.2fs) not faster than Run1 (%.2fs)\n", run2_time,
                      run1_time);
    pass = false;
  }

  if (pass) {
    (void)std::printf("test_sim: ALL PASS\n");
  }
  return pass ? 0 : 1;
}
