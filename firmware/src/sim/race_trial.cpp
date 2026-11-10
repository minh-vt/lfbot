#include "race_trial.hpp"

namespace lf::sim {

std::size_t RaceSummary::completed_count() const noexcept {
  std::size_t count = 0;
  for (const auto& record : runs) {
    if (record.started && record.verdict == VerdictKind::kCompleted) {
      ++count;
    }
  }
  return count;
}

std::size_t RaceSummary::dnf_count() const noexcept {
  std::size_t count = 0;
  for (const auto& record : runs) {
    if (record.started && record.verdict == VerdictKind::kDnf) {
      ++count;
    }
  }
  return count;
}

const RunRecord* RaceSummary::best_run() const noexcept {
  const RunRecord* best = nullptr;
  for (const auto& record : runs) {
    if (!record.started || record.verdict != VerdictKind::kCompleted) {
      continue;
    }
    if (best == nullptr || record.elapsed_us < best->elapsed_us) {
      best = &record;
    }
  }
  return best;
}

bool RaceSummary::has_gain() const noexcept {
  return runs[0].started && runs[0].verdict == VerdictKind::kCompleted && runs[1].started &&
         runs[1].verdict == VerdictKind::kCompleted;
}

std::int64_t RaceSummary::gain_us() const noexcept {
  if (!has_gain()) {
    return 0;
  }
  return static_cast<std::int64_t>(runs[1].elapsed_us) -
         static_cast<std::int64_t>(runs[0].elapsed_us);
}

RaceTrial::RaceTrial(Robot& robot, Simulation& simulation) noexcept
  : robot_(robot), simulation_(simulation) {
  trace_.reserve(kMaxTraceFrames);
  const auto& world = simulation_.observe();
  const MotorOutput disabled;
  current_ = make_frame(world, world.time_us, disabled, FrameKind::kIdle, 0);
}

void RaceTrial::reset() noexcept {
  simulation_.reset();
  const auto& world = simulation_.observe();
  const MotorOutput disabled = robot_.drive(world.measurements, RaceRequest::kReset);
  trace_.clear();
  summary_ = {};
  current_ = make_frame(world, world.time_us, disabled, FrameKind::kIdle, 0);
}

TrialFrame RaceTrial::make_frame(const WorldObservation& world, std::uint64_t decision_us,
                                 const MotorOutput& output, FrameKind kind,
                                 std::uint8_t run) const noexcept {
  TrialFrame frame;
  frame.tick = world.tick;
  frame.time_us = decision_us;
  frame.outcome_time_us = world.time_us;
  frame.track_generation = world.world_generation;
  frame.run = run;
  frame.kind = kind;
  frame.pose = world.pose;
  frame.measured_tof = world.measurements.tof[0];
  frame.output = output;
  const RobotObservation& telemetry = robot_.observe();
  frame.status = telemetry.status;
  frame.ir = telemetry.ir;
  frame.tof = telemetry.tof;
  frame.verdict = world.result.verdict;
  return frame;
}

void RaceTrial::record(const WorldObservation& world, std::uint64_t decision_us,
                       const MotorOutput& output, FrameKind kind, std::uint8_t run) noexcept {
  current_ = make_frame(world, decision_us, output, kind, run);
  if (trace_.size() < kMaxTraceFrames) {
    trace_.push_back(current_);
  }
}

void RaceTrial::record_run(const RefereeResult& result) noexcept {
  if (result.run == 0 || result.run > summary_.runs.size()) {
    return;
  }
  RunRecord& record = summary_.runs[result.run - 1U];
  record.started = true;
  record.run = result.run;
  record.verdict = result.verdict;
  record.elapsed_us = result.elapsed_us;
}

const TrialFrame& RaceTrial::step(RaceRequest request) noexcept {
  if (simulation_.track() == nullptr) {
    return current_;
  }
  if (request == RaceRequest::kReset) {
    reset();
    return current_;
  }
  if (!simulation_.running() && request == RaceRequest::kNone) {
    return current_;
  }

  if (request == RaceRequest::kStart && !simulation_.running()) {
    if (summary_.started >= summary_.runs.size() || !simulation_.prepare_run()) {
      return current_;
    }
    robot_.prepare_run();
    const WorldObservation prepared = simulation_.observe();
    const MotorOutput output = robot_.drive(prepared.measurements, RaceRequest::kStart);
    if (robot_.observe().status.phase != RacePhase::kRunning) {
      // A rejected start is not a run: no clock, no verdict, no DNF.
      record(prepared, prepared.time_us, output, FrameKind::kDecision, 0);
      return current_;
    }
    simulation_.start_run();
    ++summary_.started;
    const auto run = static_cast<std::uint8_t>(summary_.started);
    record(prepared, prepared.time_us, output, FrameKind::kDecision, run);
    const WorldObservation& after = simulation_.advance(output);
    if (after.result.verdict != VerdictKind::kPending) {
      robot_.finish_run();
      record(after, after.time_us, output, FrameKind::kTerminal, run);
      record_run(after.result);
    }
    return current_;
  }

  const WorldObservation world = simulation_.observe();
  const auto run = simulation_.run();
  const MotorOutput output = robot_.drive(world.measurements, request);
  record(world, world.time_us, output, FrameKind::kDecision, run);
  if (request == RaceRequest::kAbort) {
    simulation_.stop_run();
    robot_.finish_run();
    record(simulation_.observe(), world.time_us, output, FrameKind::kTerminal, run);
    record_run(simulation_.result());
    return current_;
  }
  if (!simulation_.running()) {
    return current_;
  }
  if (robot_.observe().status.phase == RacePhase::kFault) {
    // Safety stopped the robot; the referee no longer has a live competitor.
    simulation_.stop_run();
    const MotorOutput disabled;
    record(simulation_.observe(), world.time_us, disabled, FrameKind::kTerminal, run);
    record_run(simulation_.result());
    return current_;
  }
  const WorldObservation& after = simulation_.advance(output);
  if (after.result.verdict != VerdictKind::kPending) {
    robot_.finish_run();
    record(after, world.time_us, output, FrameKind::kTerminal, run);
    record_run(after.result);
  }
  return current_;
}

}  // namespace lf::sim
