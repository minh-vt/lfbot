#include <raylib.h>

#include <algorithm>

#include "kernel/mission.hpp"
#include "race_arbiter.hpp"
#include "renderer.hpp"
#include "track_asset.hpp"
#include "world_physics.hpp"

namespace {

constexpr const char* kManifestPath = "assets/track_vongloai.json";

struct MissionRun {
  lf::kernel::RunMode mode = lf::kernel::RunMode::kIdle;
  float elapsed = 0.0f;
  float run1_time = 0.0f;
  float run2_time = 0.0f;
};

// Một tick vật lý + điều khiển (Run 1: ghi plan; chạm finish: chuyển mode).
void advance_run(MissionRun& mission, lf::sim::WorldPhysics& world, lf::sim::RaceArbiter& arbiter,
                 const lf::sim::TrackAsset& track, lf::kernel::KernelState& state,
                 lf::kernel::TrackPlan& plan, const lf::kernel::TuneConfig& cfg, float dt) {
  mission.elapsed += dt;

  lf::kernel::SensorFrame frame;
  world.sample(frame, dt, track);
  const lf::kernel::MotorCommand command =
    lf::kernel::mission_step(state, frame, plan, mission.mode, cfg);
  world.step(command, dt);
  arbiter.observe(world.x(), world.y(), world.v(), frame.line_detected, dt);

  const lf::kernel::MissionEvent event =
    lf::kernel::advance_mission(plan, state, mission.mode, cfg);
  if (event == lf::kernel::MissionEvent::kRun1Done) {
    mission.run1_time = mission.elapsed;
    world.reset(track.manifest());
    arbiter.reset(track.manifest());
    mission.elapsed = 0.0f;
  } else if (event == lf::kernel::MissionEvent::kRun2Done) {
    mission.run2_time = mission.elapsed;
  }
}

}

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
  MissionRun mission;

  sim::Renderer renderer;
  renderer.init(track.manifest());

  while (!sim::Renderer::should_close()) {
    const float dt = std::min(GetFrameTime(), 0.01f);

    if (IsKeyPressed(KEY_R)) {
      world.reset(track.manifest());
      arbiter.reset(track.manifest());
      state = {};
      plan.reset();
      mission = {};
    }
    if (IsKeyPressed(KEY_SPACE) && mission.mode == kernel::RunMode::kIdle) {
      mission.mode = kernel::RunMode::kRun1;
    }
    if (IsKeyPressed(KEY_ONE)) {
      mission.mode = kernel::RunMode::kRun1;
    }
    if (IsKeyPressed(KEY_TWO)) {
      mission.mode = kernel::RunMode::kRun2;
    }

    if (mission.mode == kernel::RunMode::kRun1 || mission.mode == kernel::RunMode::kRun2) {
      advance_run(mission, world, arbiter, track, state, plan, cfg, dt);
    }

    const sim::Verdict verdict = arbiter.verdict();
    sim::Renderer::begin_frame();
    renderer.draw(track.manifest(), world.x(), world.y(), world.theta(),
                  static_cast<int>(mission.mode), mission.elapsed, world.v(), mission.run1_time,
                  mission.run2_time, verdict.infraction != sim::Infraction::kNone);
    sim::Renderer::end_frame();
  }

  renderer.shutdown();
  return 0;
}
