#include <raylib.h>

#include <algorithm>
#include <array>
#include <fstream>
#include <nlohmann/json.hpp>
#include <vector>

#include "drive/plan.hpp"
#include "drive/step.hpp"
#include "renderer.hpp"
#include "sensing/layout.hpp"
#include "speed_limit_calibration.hpp"
#include "track_asset.hpp"
#include "track_rules.hpp"
#include "tuning_config.hpp"
#include "world_physics.hpp"

namespace {

constexpr std::array<const char*, 2> kManifestPaths = {
  "assets/track_vongloai.json",
  "assets/track_chungket.json",
};

constexpr const char* kTuningPath = "assets/tuning.json";

struct MissionRun {
  lf::drive::RunMode mode = lf::drive::RunMode::kIdle;
  float elapsed = 0.0f;
  float run1_time = 0.0f;
  float run2_time = 0.0f;
};

// Điền các trường lấy từ vật lý (dùng chung cho trace đã ghi + trace live).
void fill_world_fields(lf::sim::TracePoint& pt, const lf::sim::WorldPhysics& world, float elapsed) {
  pt.x = world.x();
  pt.y = world.y();
  pt.theta = world.theta();
  pt.v = world.v();
  pt.tof_front = world.tof_front();
  pt.tof_left = world.tof_left();
  pt.tof_right = world.tof_right();
  pt.ir_weights = world.ir_weights();
  pt.elapsed_s = elapsed;
}

// Một tick vật lý + điều khiển (TrackRules phán quyết hoàn thành / vi phạm).
void advance_run(MissionRun& mission, lf::sim::WorldPhysics& world, lf::sim::TrackRules& rules,
                 const lf::sim::TrackAsset& track, lf::drive::DriveState& state,
                 lf::drive::RacePlan& plan, const lf::drive::Tuning& cfg,
                 std::vector<lf::sim::TracePoint>& run1_trace,
                 std::vector<lf::sim::TracePoint>& run2_trace, float dt) {
  mission.elapsed += dt;

  lf::drive::Readings frame;
  world.sample(frame, dt, track);
  const auto result = lf::drive::step(state, frame, plan, cfg);
  state = result.state;

  const float tick = 0.5f * (static_cast<float>(frame.enc_l) + static_cast<float>(frame.enc_r));
  if (mission.mode == lf::drive::RunMode::kRun1) {
    lf::drive::record(plan, tick, result.state.line_estimate, result.state.run1_speed,
                      frame.obstacle_side);
  }
  world.step(result.command, dt);
  rules.check(world.x(), world.y(), world.v(), frame.line_certainty > 0.5f, dt);

  // Thu thập trace point
  lf::sim::TracePoint pt;
  fill_world_fields(pt, world, mission.elapsed);
  pt.line_offset = frame.line_offset;
  pt.line_certainty = frame.line_certainty;
  pt.left_duty = result.command.left_duty;
  pt.right_duty = result.command.right_duty;
  pt.tick = tick;
  pt.speed_limit = plan.ready ? plan.speed_limit_at(tick) : 1.0f;
  pt.obstacle_side = plan.ready ? plan.obstacle_at(tick) : frame.obstacle_side;
  pt.line_curve_val = plan.ready ? plan.speed_profile_at(tick) : result.path_steer;

  if (mission.mode == lf::drive::RunMode::kRun1) {
    run1_trace.push_back(pt);
  } else if (mission.mode == lf::drive::RunMode::kRun2) {
    run2_trace.push_back(pt);
  }

  // Khi va chạm hoặc vi phạm: dừng ngay lập tức, giữ nguyên hiện trường
  const auto verdict = rules.verdict();
  if (verdict.infraction != lf::sim::Infraction::kNone) {
    mission.mode = lf::drive::RunMode::kDone;
    return;
  }

  // Kết thúc Run 1 hoặc Run 2 khi TrackRules công nhận hoàn thành (xe dừng trong đích)
  if (mission.mode == lf::drive::RunMode::kRun1 && verdict.completed) {
    mission.run1_time = verdict.lap_time_s;
    lf::drive::plan_speed(plan, cfg);

    for (auto& trace_pt : run1_trace) {
      trace_pt.speed_limit = plan.speed_limit_at(trace_pt.tick);
      trace_pt.obstacle_side = plan.obstacle_at(trace_pt.tick);
    }

    world.reset(track.manifest());
    rules.reset(track.manifest());
    state = {};
    state.line_estimate = plan.lateral_at(0.0f);  // Run 2 khớp setpoint ngay (tránh cú giật)
    mission.elapsed = 0.0f;
    mission.mode = lf::drive::RunMode::kRun2;
  } else if (mission.mode == lf::drive::RunMode::kRun2 && verdict.completed) {
    mission.run2_time = verdict.lap_time_s;
    mission.mode = lf::drive::RunMode::kDone;
  }
}

const std::vector<lf::sim::TracePoint>* get_active_trace(
  const MissionRun& mission, const std::vector<lf::sim::TracePoint>& run1_trace,
  const std::vector<lf::sim::TracePoint>& run2_trace) {
  if (mission.mode == lf::drive::RunMode::kRun2) {
    return &run2_trace;
  }
  if (mission.mode == lf::drive::RunMode::kDone && !run2_trace.empty()) {
    return &run2_trace;
  }
  return &run1_trace;
}

void reset_simulation(lf::sim::WorldPhysics& world, lf::sim::TrackRules& rules,
                      const lf::sim::TrackAsset& track, lf::drive::DriveState& state,
                      lf::drive::RacePlan& plan, MissionRun& mission,
                      std::vector<lf::sim::TracePoint>& run1_trace,
                      std::vector<lf::sim::TracePoint>& run2_trace, bool& paused,
                      int& scrub_frame) {
  world.reset(track.manifest());
  rules.reset(track.manifest());
  state = {};
  plan.reset();
  mission = {};
  run1_trace.clear();
  run2_trace.clear();
  paused = false;
  scrub_frame = -1;
}

void handle_scrub_input(const std::vector<lf::sim::TracePoint>& trace, bool& paused,
                        int& scrub_frame) {
  const bool step_back = IsKeyPressed(KEY_LEFT) || IsKeyPressedRepeat(KEY_LEFT);
  const bool step_fwd = IsKeyPressed(KEY_RIGHT) || IsKeyPressedRepeat(KEY_RIGHT);
  if ((!step_back && !step_fwd) || trace.empty()) {
    return;
  }

  paused = true;
  const int step = (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) ? 10 : 1;
  const int max_frame = static_cast<int>(trace.size()) - 1;

  if (scrub_frame < 0) {
    scrub_frame = max_frame;
  }
  if (step_back) {
    scrub_frame = std::max(0, scrub_frame - step);
  }
  if (step_fwd) {
    scrub_frame = std::min(max_frame, scrub_frame + step);
  }
}

void handle_playback_speed(float& playback_speed) {
  if (IsKeyPressed(KEY_COMMA)) {
    if (playback_speed > 0.5f) {
      playback_speed = 0.5f;
    } else if (playback_speed > 0.25f) {
      playback_speed = 0.25f;
    }
  }
  if (IsKeyPressed(KEY_PERIOD)) {
    if (playback_speed < 1.0f) {
      playback_speed = 1.0f;
    } else if (playback_speed < 2.0f) {
      playback_speed = 2.0f;
    }
  }
}

void poll_keyboard_input(std::size_t& track_idx, lf::sim::TrackAsset& track,
                         lf::sim::WorldPhysics& world, lf::sim::TrackRules& rules,
                         lf::drive::DriveState& state, lf::drive::RacePlan& plan,
                         MissionRun& mission, std::vector<lf::sim::TracePoint>& run1_trace,
                         std::vector<lf::sim::TracePoint>& run2_trace, lf::sim::Renderer& renderer,
                         bool& paused, float& playback_speed, int& scrub_frame) {
  if (IsKeyPressed(KEY_TAB)) {
    track_idx = (track_idx + 1) % kManifestPaths.size();
    if (track.load(kManifestPaths[track_idx])) {
      reset_simulation(world, rules, track, state, plan, mission, run1_trace, run2_trace, paused,
                       scrub_frame);
      renderer.reload_track(track.manifest());
    }
  }

  if (IsKeyPressed(KEY_R)) {
    reset_simulation(world, rules, track, state, plan, mission, run1_trace, run2_trace, paused,
                     scrub_frame);
  }

  if (IsKeyPressed(KEY_C)) {
    renderer.toggle_camera_mode();
  }

  if (IsKeyPressed(KEY_F11) || IsKeyPressed(KEY_F)) {
    ToggleFullscreen();
  }

  if (IsKeyPressed(KEY_SPACE)) {
    if (scrub_frame >= 0) {
      scrub_frame = -1;
      paused = false;
    } else if (mission.mode == lf::drive::RunMode::kIdle) {
      mission.mode = lf::drive::RunMode::kRun1;
      paused = false;
    } else {
      paused = !paused;
    }
  }

  if (IsKeyPressed(KEY_ONE)) {
    mission.mode = lf::drive::RunMode::kRun1;
    paused = false;
    scrub_frame = -1;
  }
  if (IsKeyPressed(KEY_TWO)) {
    mission.mode = lf::drive::RunMode::kRun2;
    paused = false;
    scrub_frame = -1;
  }

  const auto* active_trace = get_active_trace(mission, run1_trace, run2_trace);
  handle_scrub_input(*active_trace, paused, scrub_frame);
  handle_playback_speed(playback_speed);
}

lf::sim::RenderFrame to_render_frame(const MissionRun& mission, const lf::sim::TrackRules& rules,
                                     const lf::sim::TracePoint& pt, bool paused,
                                     float playback_speed, int scrub_frame, int total_frames) {
  lf::sim::RenderFrame frame;
  frame.mode = static_cast<int>(mission.mode);
  frame.lap1_s = mission.run1_time;
  frame.lap2_s = mission.run2_time;

  const auto verdict = rules.verdict();
  frame.dq = (verdict.infraction != lf::sim::Infraction::kNone);
  frame.dq_reason = (verdict.infraction != lf::sim::Infraction::kNone)
                      ? lf::sim::infraction_name(verdict.infraction)
                      : nullptr;
  frame.paused = paused;
  frame.playback_speed = playback_speed;
  frame.scrub_frame = scrub_frame;
  frame.total_frames = total_frames;

  frame.x = pt.x;
  frame.y = pt.y;
  frame.theta = pt.theta;
  frame.v = pt.v;
  frame.curvature = pt.line_curve_val;
  frame.speed_limit = pt.speed_limit;
  frame.line_offset = pt.line_offset;
  frame.line_certainty = pt.line_certainty;
  frame.tof_front = pt.tof_front;
  frame.tof_left = pt.tof_left;
  frame.tof_right = pt.tof_right;
  frame.ir_weights = pt.ir_weights;
  frame.left_duty = pt.left_duty;
  frame.right_duty = pt.right_duty;
  frame.elapsed_s = pt.elapsed_s;
  return frame;
}

lf::sim::TracePoint make_live_trace_point(const lf::sim::WorldPhysics& world, float elapsed) {
  lf::sim::TracePoint pt;
  fill_world_fields(pt, world, elapsed);
  return pt;
}

}  // namespace

int main() {
  using namespace lf;

  std::size_t track_idx = 0;
  sim::TrackAsset track;
  if (!track.load(kManifestPaths[track_idx])) {
    TraceLog(LOG_ERROR, "Cannot load track: %s", kManifestPaths[track_idx]);
    return 1;
  }

  sim::WorldPhysics world;
  world.reset(track.manifest());
  sim::TrackRules rules;
  rules.reset(track.manifest());

  drive::Tuning cfg;
  lf::sim::load_tuning_config(kTuningPath, cfg);
  // Hình học không-thứ-nguyên suy trực tiếp từ layout SSoT (không qua JSON).
  cfg.robot.geometry_l = sensing::geometry_l(sensing::kRobotLayout);
  cfg.robot.geometry_s = sensing::geometry_s(sensing::kRobotLayout);
  cfg.robot.speed_limit_table = sim::calibrate_speed_limit(
    cfg.robot, sim::kMaxWheelSpeedMps * sensing::ticks_per_meter(sensing::kRobotLayout));

  drive::DriveState state;
  drive::RacePlan plan;
  MissionRun mission;

  std::vector<sim::TracePoint> run1_trace;
  std::vector<sim::TracePoint> run2_trace;
  run1_trace.reserve(20000);
  run2_trace.reserve(20000);

  bool paused = false;
  float playback_speed = 1.0f;
  int scrub_frame = -1;

  sim::Renderer renderer;
  renderer.init(track.manifest());

  while (!sim::Renderer::should_close()) {
    const float raw_dt = std::min(GetFrameTime(), 0.02f);
    const float dt = raw_dt * playback_speed;

    if (IsKeyPressed(KEY_U)) {
      lf::sim::load_tuning_config(kTuningPath, cfg);
    }

    poll_keyboard_input(track_idx, track, world, rules, state, plan, mission, run1_trace,
                        run2_trace, renderer, paused, playback_speed, scrub_frame);

    // Xử lý bước vật lý khi không pause, chưa xong và không đang tua frame
    if (!paused && scrub_frame < 0 &&
        (mission.mode == drive::RunMode::kRun1 || mission.mode == drive::RunMode::kRun2)) {
      advance_run(mission, world, rules, track, state, plan, cfg, run1_trace, run2_trace, dt);
    }

    const auto& active_trace = *get_active_trace(mission, run1_trace, run2_trace);
    const int total_frames = static_cast<int>(active_trace.size());
    sim::TracePoint display_pt;
    if (scrub_frame >= 0 && !active_trace.empty()) {
      const auto idx = static_cast<std::size_t>(std::clamp(scrub_frame, 0, total_frames - 1));
      display_pt = active_trace[idx];
    } else if (!active_trace.empty()) {
      display_pt = active_trace.back();
    } else {
      display_pt = make_live_trace_point(world, mission.elapsed);
    }

    const sim::RenderFrame r_frame = to_render_frame(mission, rules, display_pt, paused,
                                                     playback_speed, scrub_frame, total_frames);

    // Cập nhật camera
    renderer.update_camera(r_frame.x, r_frame.y, r_frame.theta, r_frame.v, raw_dt, track.manifest(),
                           r_frame.scrub_frame >= 0);

    // Vẽ frame
    sim::Renderer::begin_frame();
    renderer.draw(track.manifest(), r_frame, plan, run1_trace, run2_trace);
    sim::Renderer::end_frame();
  }

  renderer.shutdown();
  return 0;
}
