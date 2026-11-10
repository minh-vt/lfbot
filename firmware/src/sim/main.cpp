#include <raylib.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <robot.hpp>
#include <string>
#include <utility>
#include <vector>

#include "race_trial.hpp"
#include "renderer.hpp"
#include "robot_geometry.hpp"
#include "simulation.hpp"
#include "track_asset.hpp"

namespace {

constexpr std::array<const char*, 2> kTrackPaths = {
  "assets/track_vongloai.json",
  "assets/track_chungket.json",
};

struct ViewState {
  bool paused = true;
  float playback_speed = 1.0F;
  float accumulator = 0.0F;
  int selected_run = 1;
  int scrub_frame = -1;
  std::size_t track_index = 0;
};

void toggle_run(const lf::sim::RaceTrial& trial, ViewState& view, lf::RaceRequest& request) {
  if (!IsKeyPressed(KEY_SPACE)) {
    return;
  }
  const auto phase = trial.frame().status.phase;
  if (view.scrub_frame >= 0) {
    view.scrub_frame = -1;
    view.paused = false;
  } else if (phase == lf::RacePhase::kIdle || phase == lf::RacePhase::kDone) {
    // A judged run leaves the robot Done; Space starts the next run when the
    // trial still allows one, otherwise the request is rejected harmlessly.
    request = lf::RaceRequest::kStart;
    view.paused = false;
  } else {
    view.paused = !view.paused;
  }
}

// Enter confirms the current run and advances to the next one, regardless of
// whether the robot completed: a running run is ended (judged DNF), an idle or
// done run starts (which also repositions the world for the next run).
void confirm_run(const lf::sim::RaceTrial& trial, ViewState& view, lf::RaceRequest& request) {
  if (!IsKeyPressed(KEY_ENTER) && !IsKeyPressed(KEY_KP_ENTER)) {
    return;
  }
  view.scrub_frame = -1;
  view.accumulator = 0.0F;
  const auto phase = trial.frame().status.phase;
  if (phase == lf::RacePhase::kRunning) {
    request = lf::RaceRequest::kAbort;
    view.paused = true;
  } else if (phase == lf::RacePhase::kIdle || phase == lf::RacePhase::kDone) {
    request = lf::RaceRequest::kStart;
    view.paused = false;
  }
}

void switch_track(lf::sim::TrackAsset& track, lf::sim::Simulation& simulation,
                  lf::sim::RaceTrial& trial, lf::sim::Renderer& renderer, ViewState& view,
                  lf::RaceRequest& request) {
  if (!IsKeyPressed(KEY_TAB)) {
    return;
  }
  const std::size_t next = (view.track_index + 1U) % kTrackPaths.size();
  lf::sim::TrackAsset candidate;
  std::string error;
  if (!candidate.load(kTrackPaths[next], error) || !renderer.reload_track(candidate)) {
    return;
  }
  track = std::move(candidate);
  simulation.use_track(track);
  trial.reset();
  view.track_index = next;
  view.scrub_frame = -1;
  view.accumulator = 0.0F;
  view.paused = true;
  request = lf::RaceRequest::kNone;
}

void seek_frame(const std::vector<lf::sim::TrialFrame>& frames, ViewState& view) {
  const bool left = IsKeyPressed(KEY_LEFT) || IsKeyPressedRepeat(KEY_LEFT);
  const bool right = IsKeyPressed(KEY_RIGHT) || IsKeyPressedRepeat(KEY_RIGHT);
  if (frames.empty() || (!left && !right)) {
    return;
  }
  view.paused = true;
  const int step = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT) ? 10 : 1;
  const int last = static_cast<int>(frames.size()) - 1;
  if (view.scrub_frame < 0) {
    view.scrub_frame = last;
  }
  if (left) {
    view.scrub_frame = std::max(0, view.scrub_frame - step);
  }
  if (right) {
    view.scrub_frame = std::min(last, view.scrub_frame + step);
  }
}

void poll_view_controls(lf::sim::TrackAsset& track, lf::sim::Simulation& simulation,
                        lf::sim::RaceTrial& trial, lf::sim::Renderer& renderer, ViewState& view,
                        lf::RaceRequest& request) {
  toggle_run(trial, view, request);
  confirm_run(trial, view, request);
  if (IsKeyPressed(KEY_ONE)) {
    view.selected_run = 1;
  }
  if (IsKeyPressed(KEY_TWO)) {
    view.selected_run = 2;
  }
  if (IsKeyPressed(KEY_R)) {
    request = lf::RaceRequest::kReset;
    view.scrub_frame = -1;
    view.accumulator = 0.0F;
    view.paused = true;
  }
  switch_track(track, simulation, trial, renderer, view, request);
  if (IsKeyPressed(KEY_C)) {
    renderer.toggle_camera_mode();
  }
  if (IsKeyPressed(KEY_F11) || IsKeyPressed(KEY_F)) {
    ToggleFullscreen();
  }
  if (IsKeyPressed(KEY_COMMA)) {
    view.playback_speed = std::max(0.25F, view.playback_speed * 0.5F);
  }
  if (IsKeyPressed(KEY_PERIOD)) {
    view.playback_speed = std::min(4.0F, view.playback_speed * 2.0F);
  }
  seek_frame(trial.trace(), view);
}

void advance_ticks(lf::sim::RaceTrial& trial, ViewState& view, lf::RaceRequest& request,
                   float render_dt) {
  if (request != lf::RaceRequest::kNone) {
    (void)trial.step(request);
    request = lf::RaceRequest::kNone;
  }
  if (view.paused) {
    return;
  }
  constexpr float kStepSeconds = static_cast<float>(lf::sim::kControlStepUs) * 1e-6F;
  view.accumulator += render_dt * view.playback_speed;
  unsigned ticks = 0;
  while (view.accumulator >= kStepSeconds && ticks < 50) {
    view.accumulator -= kStepSeconds;
    (void)trial.step(lf::RaceRequest::kNone);
    if (!trial.running()) {
      view.paused = true;
      view.accumulator = 0.0F;
      break;
    }
    ++ticks;
  }
}

const lf::sim::TrialFrame& displayed_frame(const lf::sim::RaceTrial& trial, const ViewState& view) {
  const auto& frames = trial.trace();
  if (frames.empty()) {
    return trial.frame();
  }
  const auto index = view.scrub_frame < 0
                       ? frames.size() - 1U
                       : static_cast<std::size_t>(
                           std::clamp(view.scrub_frame, 0, static_cast<int>(frames.size()) - 1));
  return frames[index];
}

class SimSession final : public lf::RobotSession {
public:
  SimSession(lf::sim::TrackAsset& track, lf::sim::Simulation& simulation,
             lf::sim::Renderer& renderer) noexcept
    : track(track), simulation(simulation), renderer(renderer) {}

  void run(lf::Robot& robot) noexcept override {
    lf::sim::RaceTrial trial(robot, simulation);
    lf::RaceRequest request = lf::RaceRequest::kNone;
    while (!lf::sim::Renderer::should_close()) {
      const float render_dt = std::max(GetFrameTime(), 0.0F);
      poll_view_controls(track, simulation, trial, renderer, state, request);
      advance_ticks(trial, state, request, render_dt);
      const auto& frame = displayed_frame(trial, state);
      renderer.update_camera(track, frame, render_dt, state.scrub_frame >= 0);
      lf::sim::Playback playback;
      playback.paused = state.paused;
      playback.seeking = state.scrub_frame >= 0;
      playback.speed = state.playback_speed;
      playback.selected_run = static_cast<std::uint8_t>(state.selected_run);
      if (state.scrub_frame >= 0) {
        playback.frame_index = static_cast<std::size_t>(state.scrub_frame);
      } else if (!trial.trace().empty()) {
        playback.frame_index = trial.trace().size() - 1U;
      }
      renderer.draw(track, frame, trial.trace(), playback);
    }
  }
  void disable_motors() noexcept override {}

private:
  lf::sim::TrackAsset& track;
  lf::sim::Simulation& simulation;
  lf::sim::Renderer& renderer;
  ViewState state;
};

}  // namespace

int main() {
  lf::sim::TrackAsset track;
  std::string error;
  if (!track.load(kTrackPaths.front(), error)) {
    TraceLog(LOG_ERROR, "Cannot load simulation track: %s", error.c_str());
    return 1;
  }
  lf::sim::Simulation simulation;
  simulation.use_track(track);
  lf::sim::Renderer renderer;
  if (!renderer.init(track)) {
    return 1;
  }
  SimSession session(track, simulation, renderer);
  lf::run_robot(lf::sim::simulated_calibration(), session);
  renderer.shutdown();
  return 0;
}
