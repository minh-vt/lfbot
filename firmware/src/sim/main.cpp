#include <algorithm>

#include "kernel/plan.hpp"
#include "kernel/step.hpp"
#include "race_arbiter.hpp"
#include <raylib.h>
#include "renderer.hpp"
#include "track_asset.hpp"
#include "world_physics.hpp"

namespace {

constexpr const char* kManifestPath = "assets/track_vongloai.json";

}  // namespace

int main() {
  using namespace lf;

  sim::TrackAsset track;
  if (!track.load(kManifestPath)) {
    TraceLog(LOG_ERROR, "Cannot load track: %s", kManifestPath);
    return 1;
  }

  sim::WorldPhysics world;
  world.reset(track.manifest());
  sim::RaceArbiter arbiter;
  arbiter.reset(track.manifest());

  const kernel::TuneConfig cfg;
  kernel::KernelState state;
  kernel::TrackPlan plan;

  sim::Renderer renderer;
  renderer.init(track.manifest());

  kernel::RunMode mode = kernel::RunMode::kIdle;
  float run_elapsed = 0.0f;
  float run1_time = 0.0f;
  float run2_time = 0.0f;

  while (!sim::Renderer::should_close()) {
    const float dt = std::min(GetFrameTime(), 0.01f);

    if (IsKeyPressed(KEY_R)) {
      world.reset(track.manifest());
      arbiter.reset(track.manifest());
      state = {};
      plan.reset();
      mode = kernel::RunMode::kIdle;
      run_elapsed = 0.0f;
      run1_time = 0.0f;
      run2_time = 0.0f;
    }
    if (IsKeyPressed(KEY_SPACE) && mode == kernel::RunMode::kIdle) {
      mode = kernel::RunMode::kRun1;
    }
    if (IsKeyPressed(KEY_ONE)) {
      mode = kernel::RunMode::kRun1;
    }
    if (IsKeyPressed(KEY_TWO)) {
      mode = kernel::RunMode::kRun2;
    }

    if (mode == kernel::RunMode::kRun1 || mode == kernel::RunMode::kRun2) {
      run_elapsed += dt;

      kernel::SensorFrame frame;
      world.sample(frame, dt, track);
      const auto result = kernel::step(state, frame, plan, cfg);
      state = result.state;

      if (mode == kernel::RunMode::kRun1) {
        const float tick =
          0.5f * (static_cast<float>(frame.enc_l) + static_cast<float>(frame.enc_r));
        kernel::record(plan, tick, result.path_steer, frame.obstacle_side);
      }

      world.step(result.command, dt);
      arbiter.observe(world.x(), world.y(), world.v(), frame.line_detected, dt);

      if (mode == kernel::RunMode::kRun1 && state.finish_latched) {
        run1_time = run_elapsed;
        kernel::bake(plan, cfg);
        world.reset(track.manifest());
        arbiter.reset(track.manifest());
        state = {};
        run_elapsed = 0.0f;
        mode = kernel::RunMode::kRun2;
      } else if (mode == kernel::RunMode::kRun2 && state.finish_latched) {
        run2_time = run_elapsed;
        mode = kernel::RunMode::kDone;
      }
    }

    const sim::Verdict verdict = arbiter.verdict();
    sim::Renderer::begin_frame();
    renderer.draw(track.manifest(), world.x(), world.y(), world.theta(), static_cast<int>(mode),
                  run_elapsed, world.v(), run1_time, run2_time,
                  verdict.infraction != sim::Infraction::kNone);
    sim::Renderer::end_frame();
  }

  renderer.shutdown();
  return 0;
}
