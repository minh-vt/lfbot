#pragma once

#include <cstdint>
#include <robot_data.hpp>

#include "referee.hpp"
#include "track_asset.hpp"
#include "world_physics.hpp"

namespace lf::sim {

inline constexpr std::uint64_t kControlStepUs = 2000;

// One published view of the simulated world. Ground truth here is for sensors,
// rendering, rewards and judging; a deployed controller must not read it.
struct WorldObservation {
  std::uint64_t tick = 0;
  std::uint64_t time_us = 0;
  std::uint32_t world_generation = 0;
  Pose pose;
  bool blocked = false;  // chassis contact stopped this tick's commanded motion
  SensorMeasurements measurements;
  RefereeResult result;
};

// Advances a borrowed track from motor outputs. It has no controller: callers
// read the observation, decide, and pass the resulting output to advance().
class Simulation {
public:
  explicit Simulation(const Calibration& calibration = simulated_calibration(),
                      const WorldParameters& parameters = {}) noexcept
    : world_(calibration, parameters), referee_(calibration) {}

  // Borrows a track for the lifetime of every run using this simulation.
  void use_track(const TrackAsset& track) noexcept;

  // Clears world/referee/run accounting on a new world generation. The clock
  // stays monotonic for the session so sensor timestamps never go backwards.
  void reset() noexcept;

  // Repositions the world at the start gate for the next run. Robot memory and
  // the session clock are untouched; no run is counted until start_run().
  [[nodiscard]] bool prepare_run() noexcept;

  // Commits the prepared run and starts the referee clock.
  void start_run() noexcept;

  // Ends a running run without a judged verdict; the caller reports the reason.
  void stop_run() noexcept;

  // Integrates one control step of the given output and publishes the result.
  // Idle calls (no prepared/running run) do not advance the clock.
  [[nodiscard]] const WorldObservation& advance(const MotorOutput& output) noexcept;

  [[nodiscard]] const WorldObservation& observe() const noexcept { return current_; }
  [[nodiscard]] const TrackAsset* track() const noexcept { return track_; }
  [[nodiscard]] bool running() const noexcept { return running_; }
  [[nodiscard]] std::uint8_t run() const noexcept { return run_; }
  [[nodiscard]] const RefereeResult& result() const noexcept { return result_; }

private:
  void publish(std::uint64_t now_us) noexcept;

  const TrackAsset* track_ = nullptr;
  WorldPhysics world_;
  Referee referee_;
  WorldObservation current_;
  RefereeResult result_;
  std::uint64_t clock_us_ = 0;
  std::uint64_t run_start_us_ = 0;
  std::uint64_t tick_ = 0;
  std::uint32_t generation_ = 0;
  std::uint8_t run_ = 0;
  bool prepared_ = false;
  bool running_ = false;
};

}  // namespace lf::sim
