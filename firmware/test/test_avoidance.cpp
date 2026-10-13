// Harness thí nghiệm cảm biến (ticket 04) + gate né vật cản.
// - Quét góc ToF sườn {0,26,45,60,90}° trên kịch bản gạch-trên-đường → chọn góc bằng SỐ LIỆU.
// - So mảng IR thẳng vs cong: độ nhạy của line_offset theo góc nghiêng vạch.
// - Gate: 45° (kRobotLayout) phải gặp vật cản VÀ không va chạm.
#include <cmath>
#include <cstdio>

#include "drive/plan.hpp"
#include "drive/step.hpp"
#include "sensing/layout.hpp"
#include "sensing/normalize.hpp"
#include "speed_limit_calibration.hpp"
#include "track_asset.hpp"
#include "track_rules.hpp"
#include "world_physics.hpp"

namespace {

constexpr const char* kAvoidManifest = "assets/track_avoid.json";
constexpr float kDt = 0.002f;      // 500Hz
constexpr int kMaxSteps = 150000;  // 300s

struct Outcome {
  float max_danger = 0.0f;
  float min_clearance_mm = 1e9f;  // khoảng cách (trừ bán kính) tới AABB vật cản
  lf::sim::Infraction infraction = lf::sim::Infraction::kNone;
  bool completed = false;
};

// Chạy kịch bản né 1 lượt với một layout cảm biến cho trước.
Outcome run_avoid(const lf::sensing::RobotLayout& layout, const lf::sim::TrackAsset& track) {
  lf::sim::WorldPhysics world(layout);
  world.reset(track.manifest());
  lf::sim::TrackRules rules;
  rules.reset(track.manifest());

  lf::drive::Tuning cfg;
  cfg.robot.geometry_l = lf::sensing::geometry_l(layout);
  cfg.robot.geometry_s = lf::sensing::geometry_s(layout);
  cfg.robot.speed_limit_table = lf::sim::calibrate_speed_limit(
    cfg.robot, lf::sim::kMaxWheelSpeedMps * lf::sensing::ticks_per_meter(layout));

  lf::drive::DriveState state;
  const lf::drive::RacePlan plan;
  const auto& obstacle = track.manifest().obstacles.front();
  const float radius = lf::sensing::robot_radius_mm(layout);

  Outcome out;
  int steps = 0;
  while (steps++ < kMaxSteps) {
    lf::drive::Readings frame;
    world.sample(frame, kDt, track);
    const auto result = lf::drive::step(state, frame, plan, cfg);
    state = result.state;
    world.step(result.command, kDt);
    rules.check(world.x(), world.y(), world.v(), frame.line_certainty > 0.5f, kDt);

    out.max_danger = std::max(out.max_danger, frame.obstacle_danger);
    const float dx = std::abs(world.x() - obstacle.x_mm) - obstacle.width_mm * 0.5f;
    const float dy = std::abs(world.y() - obstacle.y_mm) - obstacle.height_mm * 0.5f;
    const float to_box =
      std::sqrt(std::max(dx, 0.0f) * std::max(dx, 0.0f) + std::max(dy, 0.0f) * std::max(dy, 0.0f));
    out.min_clearance_mm = std::min(out.min_clearance_mm, to_box - radius);

    const auto verdict = rules.verdict();
    if (verdict.infraction != lf::sim::Infraction::kNone || verdict.completed) {
      out.infraction = verdict.infraction;
      out.completed = verdict.completed;
      break;
    }
  }
  return out;
}

// Mô hình vạch đơn giản (vạch rộng 20mm): cảm biến i đọc tối nếu tâm vạch ở forward_i
// (y = y0 + forward_i·tan(psi)) nằm trong bán kính vạch của nó. Dùng để so thẳng vs cong.
lf::sensing::IrReadings synth_line(const lf::sensing::RobotLayout& layout, float y0, float psi) {
  constexpr float kHalfLineMm = 10.0f;
  lf::sensing::IrReadings ir;
  for (std::size_t i = 0; i < layout.ir_lateral_mm.size(); ++i) {
    const float line_y = y0 + layout.ir_forward_mm[i] * std::tan(psi);
    ir.raw[i] = (std::abs(line_y - layout.ir_lateral_mm[i]) < kHalfLineMm) ? 0 : 4095;
  }
  return ir;
}

// Độ nhạy line_offset theo góc nghiêng vạch (straight vs curved).
float offset_at(const lf::sensing::RobotLayout& layout, float psi) {
  return lf::sensing::normalize_ir(synth_line(layout, 0.0f, psi), layout).line_offset;
}

}

int main() {
  using namespace lf;

  // Chống regression: đổi màn KHÔNG để lại vật cản của màn cũ (manifest phải reset mỗi lần load).
  {
    sim::TrackAsset probe;
    if (!probe.load("assets/track_avoid.json") || !probe.load("assets/track_vongloai.json")) {
      (void)std::printf("test_avoidance: FAIL cannot load probe tracks\n");
      return 1;
    }
    if (probe.manifest().obstacles.size() != 2) {
      (void)std::printf("test_avoidance: FAIL map reload leaked obstacles (%zu)\n",
                        probe.manifest().obstacles.size());
      return 1;
    }
  }

  sim::TrackAsset track;
  if (!track.load(kAvoidManifest)) {
    (void)std::printf("test_avoidance: FAIL cannot load track\n");
    return 1;
  }

  // --- T04a: quét góc ToF sườn ---
  (void)std::printf("ToF side-angle sweep (brick on Run-1 path):\n");
  (void)std::printf("  %-7s %-8s %-11s %s\n", "angle", "danger", "clearance", "outcome");
  for (const float angle : {0.0f, 26.0f, 45.0f, 60.0f, 90.0f}) {
    sensing::RobotLayout layout = sensing::kRobotLayout;
    layout.tof_side_angle_deg = angle;
    const Outcome out = run_avoid(layout, track);
    (void)std::printf("  %5.0f° %8.2f %9.0fmm %s\n", angle, out.max_danger, out.min_clearance_mm,
                      sim::infraction_name(out.infraction));
  }

  // --- T04b: mảng IR thẳng vs cong (độ nhạy offset theo góc nghiêng vạch) ---
  sensing::RobotLayout curved = sensing::kRobotLayout;
  for (std::size_t i = 0; i < curved.ir_forward_mm.size(); ++i) {
    curved.ir_forward_mm[i] += 0.5f * std::abs(curved.ir_lateral_mm[i]);  // đầu cong về trước
  }
  (void)std::printf("IR array: offset tại psi=0 / psi=0.10rad (thẳng vs cong):\n");
  (void)std::printf("  straight: %+.3f / %+.3f   curved: %+.3f / %+.3f\n",
                    offset_at(sensing::kRobotLayout, 0.0f), offset_at(sensing::kRobotLayout, 0.1f),
                    offset_at(curved, 0.0f), offset_at(curved, 0.1f));

  // --- Gate: layout hiện tại (45°) phải gặp vật cản + không va chạm ---
  const Outcome gate = run_avoid(sensing::kRobotLayout, track);
  (void)std::printf(
    "test_avoidance: steps=max max_danger=%.2f min_clearance=%.0fmm infraction=%s completed=%d\n",
    gate.max_danger, gate.min_clearance_mm, sim::infraction_name(gate.infraction),
    gate.completed ? 1 : 0);
  if (gate.max_danger < 0.5f) {
    (void)std::printf("test_avoidance: FAIL never encountered obstacle (vacuous test)\n");
    return 1;
  }
  if (gate.infraction == sim::Infraction::kObstacleCollision) {
    (void)std::printf("test_avoidance: FAIL obstacle collision\n");
    return 1;
  }
  (void)std::printf("test_avoidance: ALL PASS (encountered + no collision)\n");
  return 0;
}
