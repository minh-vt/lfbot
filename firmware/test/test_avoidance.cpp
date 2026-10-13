// Kiểm thử né vật cản headless: gạch đặt ĐÚNG trên đường chạy Run 1.
// Gate: robot PHẢI gặp vật cản (danger vượt ngưỡng) VÀ KHÔNG va chạm.
#include <cstdio>

#include "drive/plan.hpp"
#include "drive/step.hpp"
#include "sensing/layout.hpp"
#include "speed_limit_calibration.hpp"
#include "track_asset.hpp"
#include "track_rules.hpp"
#include "world_physics.hpp"

namespace {

constexpr const char* kManifestPath = "assets/track_avoid.json";
constexpr float kDt = 0.002f;      // 500Hz
constexpr int kMaxSteps = 150000;  // 300s

}

int main() {
  using namespace lf;

  sim::TrackAsset track;
  if (!track.load(kManifestPath)) {
    (void)std::printf("test_avoidance: FAIL cannot load track\n");
    return 1;
  }

  sim::WorldPhysics world;
  world.reset(track.manifest());
  sim::TrackRules rules;
  rules.reset(track.manifest());

  drive::Tuning cfg;
  cfg.robot.geometry_l = sensing::geometry_l(sensing::kRobotLayout);
  cfg.robot.geometry_s = sensing::geometry_s(sensing::kRobotLayout);
  cfg.robot.speed_limit_table = sim::calibrate_speed_limit(
    cfg.robot, sim::kMaxWheelSpeedMps * sensing::ticks_per_meter(sensing::kRobotLayout));

  drive::DriveState state;
  const drive::RacePlan plan;

  int steps = 0;
  float max_danger = 0.0f;
  float min_clearance = 1e9f;
  const auto& obstacle = track.manifest().obstacles.front();

  while (steps++ < kMaxSteps) {
    drive::Readings frame;
    world.sample(frame, kDt, track);
    const auto result = drive::step(state, frame, plan, cfg);
    state = result.state;
    world.step(result.command, kDt);
    rules.check(world.x(), world.y(), world.v(), frame.line_certainty > 0.5f, kDt);

    max_danger = std::max(max_danger, frame.obstacle_danger);
    const float dx = std::abs(world.x() - obstacle.x_mm) - obstacle.width_mm * 0.5f;
    const float dy = std::abs(world.y() - obstacle.y_mm) - obstacle.height_mm * 0.5f;
    const float to_box =
      std::sqrt(std::max(dx, 0.0f) * std::max(dx, 0.0f) + std::max(dy, 0.0f) * std::max(dy, 0.0f));
    min_clearance = std::min(min_clearance, to_box);

    const auto verdict = rules.verdict();
    if (verdict.infraction != sim::Infraction::kNone || verdict.completed) {
      break;
    }
  }

  const auto verdict = rules.verdict();
  const float radius = sensing::robot_radius_mm(sensing::kRobotLayout);
  (void)std::printf(
    "test_avoidance: steps=%d max_danger=%.2f min_clearance=%.0fmm infraction=%s completed=%d\n",
    steps, max_danger, min_clearance - radius, sim::infraction_name(verdict.infraction),
    verdict.completed ? 1 : 0);

  if (max_danger < 0.5f) {
    (void)std::printf("test_avoidance: FAIL never encountered obstacle (vacuous test)\n");
    return 1;
  }
  if (verdict.infraction == sim::Infraction::kObstacleCollision) {
    (void)std::printf("test_avoidance: FAIL obstacle collision\n");
    return 1;
  }
  (void)std::printf("test_avoidance: ALL PASS (encountered + no collision)\n");
  return 0;
}
