#include <cstdio>

#include "kernel/mission.hpp"
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

  const kernel::TuneConfig cfg;
  kernel::KernelState state;
  kernel::TrackPlan plan;
  kernel::RunMode mode = kernel::RunMode::kRun1;
  float run_elapsed = 0.0f;
  float run1_time = 0.0f;
  float run2_time = 0.0f;
  sim::Verdict run1_verdict;
  sim::Verdict run2_verdict;

  int steps = 0;
  while (steps++ < kMaxSteps && mode != kernel::RunMode::kDone) {
    kernel::SensorFrame frame;
    world.sample(frame, kDt, track);
    if (steps % 2000 == 0) {
      (void)std::printf("  [s=%5d t=%.0fs] pos=(%.0f,%.0f) th=%3.0f err=%+.2f line=%d\n", steps,
                        run_elapsed, world.x(), world.y(), world.theta() * (180.0f / sim::kPi),
                        frame.line_error, frame.line_detected ? 1 : 0);
    }
    const kernel::MotorCommand command = kernel::mission_step(state, frame, plan, mode, cfg);
    world.step(command, kDt);
    arbiter.observe(world.x(), world.y(), world.v(), frame.line_detected, kDt);
    run_elapsed += kDt;

    const kernel::MissionEvent event = kernel::advance_mission(plan, state, mode, cfg);
    if (event == kernel::MissionEvent::kRun1Done) {
      run1_verdict = arbiter.verdict();
      run1_time = run_elapsed;
      world.reset(track.manifest());
      arbiter.reset(track.manifest());
      run_elapsed = 0.0f;
    } else if (event == kernel::MissionEvent::kRun2Done) {
      run2_verdict = arbiter.verdict();
      run2_time = run_elapsed;
    }
  }

  (void)std::printf("test_sim: steps=%d  Run1=%.2fs (%s)  Run2=%.2fs (%s)  plan=%zu ticks\n", steps,
                    run1_time, run1_verdict.completed ? "OK" : "DQ", run2_time,
                    run2_verdict.completed ? "OK" : "DQ", plan.steer_ff.size());

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
