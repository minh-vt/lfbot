#include <raylib.h>

#include <algorithm>

#include "drive/plan.hpp"
#include "drive/step.hpp"
#include "race_arbiter.hpp"
#include "renderer.hpp"
#include "track_asset.hpp"
#include "world_physics.hpp"

namespace {

constexpr const char* kManifestPath = "assets/track_vongloai.json";

struct MissionRun {
  lf::drive::RunMode mode = lf::drive::RunMode::kIdle;
  float elapsed = 0.0f;
  float run1_time = 0.0f;
  float run2_time = 0.0f;
};

// Một tick vật lý + điều khiển (Run 1: ghi plan; chạm finish: chuyển mode).
void advance_run(MissionRun& mission, lf::sim::WorldPhysics& world, lf::sim::RaceArbiter& arbiter,
                 const lf::sim::TrackAsset& track, lf::drive::DriveState& state,
                 lf::drive::RacePlan& plan, const lf::drive::Tuning& cfg, float dt) {
  mission.elapsed += dt;

  lf::drive::Readings frame;
  world.sample(frame, dt, track);
  const auto result = lf::drive::step(state, frame, plan, cfg);
  state = result.state;
  if (mission.mode == lf::drive::RunMode::kRun1) {
    const float tick = 0.5f * (static_cast<float>(frame.enc_l) + static_cast<float>(frame.enc_r));
    lf::drive::record(plan, tick, result.path_steer, frame.obstacle_side);
  }
  world.step(result.command, dt);
  arbiter.observe(world.x(), world.y(), world.v(), frame.line_certainty > 0.5f, dt);

  if (mission.mode == lf::drive::RunMode::kRun1 && state.finish_latched) {
    mission.run1_time = mission.elapsed;
    lf::drive::plan_speed(plan, cfg);
    world.reset(track.manifest());
    arbiter.reset(track.manifest());
    state = {};
    mission.elapsed = 0.0f;
    mission.mode = lf::drive::RunMode::kRun2;
  } else if (mission.mode == lf::drive::RunMode::kRun2 && state.finish_latched) {
    mission.run2_time = mission.elapsed;
    mission.mode = lf::drive::RunMode::kDone;
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

  const drive::Tuning cfg;
  drive::DriveState state;
  drive::RacePlan plan;
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
    if (IsKeyPressed(KEY_SPACE) && mission.mode == drive::RunMode::kIdle) {
      mission.mode = drive::RunMode::kRun1;
    }
    if (IsKeyPressed(KEY_ONE)) {
      mission.mode = drive::RunMode::kRun1;
    }
    if (IsKeyPressed(KEY_TWO)) {
      mission.mode = drive::RunMode::kRun2;
    }

    if (mission.mode == drive::RunMode::kRun1 || mission.mode == drive::RunMode::kRun2) {
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
