#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <robot.hpp>
#include <vector>

#include "simulation.hpp"
#include "trial_frame.hpp"

namespace lf::sim {

struct RunRecord {
  bool started = false;
  std::uint8_t run = 0;
  VerdictKind verdict = VerdictKind::kPending;
  std::uint64_t elapsed_us = 0;
};

// Accounting across the competition runs. Only a judged run is a result; a
// start that the robot rejected stays unrecorded instead of becoming a DNF.
struct RaceSummary {
  std::array<RunRecord, 2> runs{};
  std::size_t started = 0;

  [[nodiscard]] std::size_t completed_count() const noexcept;
  [[nodiscard]] std::size_t dnf_count() const noexcept;
  // Fastest completed run, or nullptr when nothing completed.
  [[nodiscard]] const RunRecord* best_run() const noexcept;
  // True only when both runs completed and a time difference is meaningful.
  [[nodiscard]] bool has_gain() const noexcept;
  // run 2 minus run 1 in microseconds; negative means the second run was faster.
  [[nodiscard]] std::int64_t gain_us() const noexcept;
};

// Composes one Robot with one Simulation for the GUI.
// It owns orchestration and the recorded trace; it never decides motion.
class RaceTrial {
public:
  RaceTrial(Robot& robot, Simulation& simulation) noexcept;
  RaceTrial(const RaceTrial&) = delete;
  RaceTrial& operator=(const RaceTrial&) = delete;
  RaceTrial(RaceTrial&&) = delete;
  RaceTrial& operator=(RaceTrial&&) = delete;
  ~RaceTrial() = default;

  // Full session reset: clears trace, run accounting and controller memory.
  void reset() noexcept;

  // One control step. The request applies to this step only.
  [[nodiscard]] const TrialFrame& step(RaceRequest request) noexcept;

  [[nodiscard]] const TrialFrame& frame() const noexcept { return current_; }
  [[nodiscard]] const std::vector<TrialFrame>& trace() const noexcept { return trace_; }
  [[nodiscard]] const RaceSummary& summary() const noexcept { return summary_; }
  [[nodiscard]] bool running() const noexcept { return simulation_.running(); }

private:
  [[nodiscard]] TrialFrame make_frame(const WorldObservation& world, std::uint64_t decision_us,
                                      const MotorOutput& output, FrameKind kind,
                                      std::uint8_t run) const noexcept;
  void record(const WorldObservation& world, std::uint64_t decision_us, const MotorOutput& output,
              FrameKind kind, std::uint8_t run) noexcept;
  void record_run(const RefereeResult& result) noexcept;

  Robot& robot_;
  Simulation& simulation_;
  std::vector<TrialFrame> trace_;
  TrialFrame current_;
  RaceSummary summary_;
};

}  // namespace lf::sim
