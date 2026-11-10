#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <race_trial.hpp>
#include <robot.hpp>
#include <robot_geometry.hpp>
#include <simulation.hpp>
#include <string>
#include <track_asset.hpp>

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::abort();
  }
}

class ObstacleSession final : public lf::RobotSession {
public:
  ObstacleSession(lf::sim::TrackAsset& track, lf::sim::Simulation& simulation)
    : track_(track), simulation_(simulation) {}

  void run(lf::Robot& robot) noexcept override {
    lf::sim::RaceTrial trial(robot, simulation_);
    auto request = lf::RaceRequest::kStart;
    bool approached = false;
    bool detected = false;
    for (unsigned tick = 0; tick < 25000; ++tick) {
      const auto& frame = trial.step(request);
      request = lf::RaceRequest::kNone;
      approached = approached || frame.pose.x_mm > track_.obstacles.front().x_mm - 300.0F;
      detected = detected || (frame.measured_tof.status == lf::ReadingStatus::kValid &&
                              frame.measured_tof.value < 500.0F);
      if (!trial.running()) {
        break;
      }
    }
    // The robot must reach and detect the brick without tripping a sensor
    // fault. Completing the full detour is still a known single-ToF limitation
    // (no lateral cue, no "past the brick" signal); the robot may stop on a
    // jam (kBlocked) or oscillate until timeout. See IMPLEMENTATION.
    require(approached, "the robot must reach the brick vicinity");
    require(detected, "the robot must detect the brick ahead");
    const auto& final = trial.frame();
    const bool sensor_fault = final.status.phase == lf::RacePhase::kFault &&
                              final.status.reason == lf::StopReason::kSensorFault;
    require(!sensor_fault, "avoidance must not trip a sensor fault");
    std::cerr << "obstacle detection-only; detour NOT completed: last x=" << final.pose.x_mm
              << " y=" << final.pose.y_mm << " phase=" << static_cast<int>(final.status.phase)
              << " reason=" << static_cast<int>(final.status.reason) << '\n';
  }
  void disable_motors() noexcept override {}

private:
  lf::sim::TrackAsset& track_;
  lf::sim::Simulation& simulation_;
};

class Session final : public lf::RobotSession {
public:
  Session(lf::sim::TrackAsset& track, lf::sim::Simulation& simulation)
    : track_(track), simulation_(simulation) {}

  void run(lf::Robot& robot) noexcept override {
    lf::sim::RaceTrial trial(robot, simulation_);
    const auto verify_start = [this](const lf::sim::TrialFrame& frame) {
      require(std::abs(frame.pose.x_mm - track_.start_x_mm) < 1e-3F &&
                std::abs(frame.pose.y_mm - track_.start_y_mm) < 1e-3F &&
                std::abs(frame.pose.yaw_rad - track_.start_yaw_rad) < 1e-3F,
              "the displayed idle frame uses the track start pose, not the default origin");
      require(frame.track_generation == simulation_.observe().world_generation,
              "the displayed frame belongs to the current world");
    };
    verify_start(trial.frame());
    verify_start(trial.step(lf::RaceRequest::kNone));
    require(trial.trace().empty() && !trial.running(), "initialization does not start a run");

    (void)trial.step(lf::RaceRequest::kStart);
    require(trial.running() && trial.frame().run == 1, "start commits run one");
    (void)trial.step(lf::RaceRequest::kStart);
    require(trial.summary().started == 1, "a repeated start does not consume another run");
    const auto& aborted = trial.step(lf::RaceRequest::kAbort);
    require(!trial.running() && aborted.verdict == lf::sim::VerdictKind::kDnf,
            "abort ends the run with a DNF");
    require(aborted.status.phase == lf::RacePhase::kDone &&
              aborted.status.reason == lf::StopReason::kAborted,
            "the robot retains the abort reason");
    require(trial.summary().runs[0].elapsed_us > 0, "abort retains measured elapsed time");

    verify_start(trial.step(lf::RaceRequest::kStart));
    require(trial.running() && trial.frame().run == 2, "run two is repositioned at start");
    (void)trial.step(lf::RaceRequest::kAbort);
    (void)trial.step(lf::RaceRequest::kStart);
    require(!trial.running() && trial.summary().started == 2, "a third start is rejected");

    const auto& reset = trial.step(lf::RaceRequest::kReset);
    verify_start(reset);
    require(
      reset.status.phase == lf::RacePhase::kIdle && reset.verdict == lf::sim::VerdictKind::kPending,
      "reset clears the status and verdict");
    require(trial.trace().empty() && trial.summary().started == 0, "reset clears the session");

    std::string error;
    require(track_.load("assets/track_chungket.json", error), error.c_str());
    simulation_.use_track(track_);
    trial.reset();
    verify_start(trial.frame());
    for (std::size_t i = 0; i < lf::kIrSensorCount; ++i) {
      const float expected = 1.0F - simulation_.observe().measurements.ir[i].value / 255.0F;
      require(std::abs(trial.frame().ir[i].value - expected) < 1e-3F,
              "track switching displays fresh IR from the new start, not the previous track");
    }
    (void)trial.step(lf::RaceRequest::kStart);
    require(trial.running() && trial.frame().run == 1, "a switched track starts at run one");
  }
  void disable_motors() noexcept override {}

private:
  lf::sim::TrackAsset& track_;
  lf::sim::Simulation& simulation_;
};

}  // namespace

int main() {
  lf::sim::TrackAsset track;
  std::string error;
  require(track.load("assets/track_vongloai.json", error), error.c_str());
  lf::sim::Simulation simulation;
  simulation.use_track(track);
  Session session(track, simulation);
  lf::run_robot(lf::sim::simulated_calibration(), session);
  require(track.load("assets/track_vongloai.json", error), error.c_str());
  lf::sim::Simulation obstacle_simulation;
  obstacle_simulation.use_track(track);
  ObstacleSession obstacle(track, obstacle_simulation);
  lf::run_robot(lf::sim::simulated_calibration(), obstacle);
  std::cout
    << "sim trial: lifecycle PASS; obstacle check is detection-only, detour NOT completed\n";
}
