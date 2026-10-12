#include <cstdio>

#include "drive/plan.hpp"
#include "drive/step.hpp"
#include "speed_limit_calibration.hpp"
#include "track_asset.hpp"
#include "track_rules.hpp"
#include "world_physics.hpp"

namespace {

constexpr const char* kManifestPath = "assets/track_vongloai.json";
constexpr float kDt = 0.002f;      // 500Hz
constexpr int kMaxSteps = 120000;  // 240s

bool verify_verdicts(const lf::sim::Verdict& run1_verdict, float run1_time,
                     const lf::sim::Verdict& run2_verdict, float run2_time) {
  if (!run1_verdict.completed) {
    (void)std::printf("test_sim: FAIL Run1 (%s)\n",
                      lf::sim::infraction_name(run1_verdict.infraction));
    return false;
  }
  if (!run2_verdict.completed) {
    (void)std::printf("test_sim: FAIL Run2 (%s)\n",
                      lf::sim::infraction_name(run2_verdict.infraction));
    return false;
  }
  if (run2_time >= run1_time) {
    (void)std::printf("test_sim: FAIL Run2 (%.2fs) not faster than Run1 (%.2fs)\n", run2_time,
                      run1_time);
    return false;
  }
  (void)std::printf("test_sim: ALL PASS\n");
  return true;
}

}  // namespace

int main() {
  using namespace lf;

  sim::TrackAsset track;
  if (!track.load(kManifestPath)) {
    (void)std::printf("test_sim: FAIL cannot load track\n");
    return 1;
  }

  sim::WorldPhysics world;
  world.reset(track.manifest());
  sim::TrackRules rules;
  rules.reset(track.manifest());

  drive::Tuning cfg;
  cfg.robot.speed_limit_table =
    sim::calibrate_speed_limit(cfg.robot, sim::kMaxWheelSpeedMps * sim::kTicksPerMeter);
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
    const auto result = drive::step(state, frame, plan, cfg);
    state = result.state;
    if (steps % 2000 == 0) {
      (void)std::printf(
        "  [s=%5d t=%.0fs] pos=(%.0f,%.0f) th=%3.0f err=%+.2f cert=%.2f v=%.2f st=%+.2f spd=%.2f\n",
        steps, run_elapsed, world.x(), world.y(), world.theta() * (180.0f / sim::kPi),
        frame.line_offset, frame.line_certainty, world.v(), result.path_steer, state.run1_speed);
    }
    if (mode == drive::RunMode::kRun1) {
      const float tick = 0.5f * (static_cast<float>(frame.enc_l) + static_cast<float>(frame.enc_r));
      drive::record(plan, tick, result.path_steer, frame.obstacle_side);
    }
    world.step(result.command, kDt);
    rules.check(world.x(), world.y(), world.v(), frame.line_certainty > 0.5f, kDt);
    run_elapsed += kDt;

    const auto verdict = rules.verdict();
    if (verdict.infraction != sim::Infraction::kNone) {
      if (mode == drive::RunMode::kRun1) {
        run1_verdict = verdict;
      } else {
        run2_verdict = verdict;
      }
      break;
    }

    if (mode == drive::RunMode::kRun1 && verdict.completed) {
      run1_verdict = verdict;
      run1_time = verdict.lap_time_s;
      drive::plan_speed(plan, cfg);
      world.reset(track.manifest());
      rules.reset(track.manifest());
      state = {};
      run_elapsed = 0.0f;
      mode = drive::RunMode::kRun2;
    } else if (mode == drive::RunMode::kRun2 && verdict.completed) {
      run2_verdict = verdict;
      run2_time = verdict.lap_time_s;
      mode = drive::RunMode::kDone;
    }
  }

  const char* v1_status = run1_verdict.completed ? "OK" : "DQ";
  const char* v2_status = run2_verdict.completed ? "OK" : "DQ";
  (void)std::printf("test_sim: steps=%d  Run1=%.2fs (%s)  Run2=%.2fs (%s)  plan=%zu ticks\n", steps,
                    run1_time, v1_status, run2_time, v2_status, plan.planned_steer.size());

  return verify_verdicts(run1_verdict, run1_time, run2_verdict, run2_time) ? 0 : 1;
}
