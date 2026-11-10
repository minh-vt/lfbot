#include "simulation.hpp"

#include <cmath>

namespace lf::sim {
namespace {

// The engine is a second trust seam: a caller that bypasses Drive must not be
// able to integrate NaN or out-of-range duty into the plant.
MotorOutput accept_output(const MotorOutput& output, std::uint64_t begin_us) noexcept {
  const bool valid = std::isfinite(output.left_duty) && std::isfinite(output.right_duty) &&
                     std::abs(output.left_duty) <= 1.0F && std::abs(output.right_duty) <= 1.0F &&
                     output.deadline_us > begin_us;
  if (valid) {
    return output;
  }
  MotorOutput disabled;
  disabled.deadline_us = begin_us;
  return disabled;
}

}  // namespace

void Simulation::use_track(const TrackAsset& track) noexcept {
  track_ = &track;
  ++generation_;
  clock_us_ = 0;
  reset();
}

void Simulation::reset() noexcept {
  if (track_ == nullptr) {
    return;
  }
  world_.reset(*track_);
  referee_.reset();
  result_ = {};
  current_ = {};
  tick_ = 0;
  run_ = 0;
  prepared_ = false;
  running_ = false;
  // Idle time is not simulated, but a new session must hand the controller a
  // timestamp later than the reset request it just received.
  clock_us_ += kControlStepUs;
  publish(clock_us_);
}

bool Simulation::prepare_run() noexcept {
  if (track_ == nullptr || !world_.parameters_valid()) {
    return false;
  }
  world_.reset(*track_);
  referee_.reset();
  result_ = {};
  prepared_ = true;
  running_ = false;
  // The repositioned world publishes one fresh packet so the controller never
  // sees two decisions at the same timestamp.
  clock_us_ += kControlStepUs;
  publish(clock_us_);
  return true;
}

void Simulation::start_run() noexcept {
  if (!prepared_ || running_) {
    return;
  }
  prepared_ = false;
  running_ = true;
  ++run_;
  run_start_us_ = clock_us_;
  referee_.start_run(clock_us_, run_);
}

void Simulation::stop_run() noexcept {
  if (!running_) {
    return;
  }
  running_ = false;
  if (result_.verdict == VerdictKind::kPending) {
    result_ = {VerdictKind::kDnf, clock_us_ - run_start_us_, run_};
    current_.result = result_;
  }
}

const WorldObservation& Simulation::advance(const MotorOutput& output) noexcept {
  if (track_ == nullptr || !running_) {
    return current_;
  }
  const MotorOutput accepted = accept_output(output, clock_us_);
  world_.advance(accepted, clock_us_, clock_us_ + kControlStepUs, *track_);
  clock_us_ += kControlStepUs;
  ++tick_;
  result_ = referee_.check(*track_, world_.pose(), clock_us_);
  if (result_.verdict != VerdictKind::kPending) {
    running_ = false;
  }
  publish(clock_us_);
  return current_;
}

void Simulation::publish(std::uint64_t now_us) noexcept {
  current_.tick = tick_;
  current_.time_us = now_us;
  current_.world_generation = generation_;
  current_.pose = world_.pose();
  current_.blocked = world_.blocked();
  current_.measurements = track_ != nullptr ? world_.sense(now_us, *track_) : SensorMeasurements{};
  current_.result = result_;
}

}  // namespace lf::sim
