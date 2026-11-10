#pragma once

#include <array>
#include <cstdint>
#include <robot_data.hpp>

#include "referee.hpp"
#include "simulation.hpp"

namespace lf::sim {

enum class FrameKind : std::uint8_t { kIdle, kDecision, kTerminal };

// One recorded observation: the decision made at time_us and the world state it
// produced. Physical measurements are kept beside normalized telemetry so the
// UI can show real units without asking the controller.
struct TrialFrame {
  std::uint64_t tick = 0;
  std::uint64_t time_us = 0;          // decision time
  std::uint64_t outcome_time_us = 0;  // world time of pose and measurements
  std::uint32_t track_generation = 0;
  std::uint8_t run = 0;
  FrameKind kind = FrameKind::kIdle;
  Pose pose;
  // Physical front range in mm for the renderer; the normalized ToF is in `tof`.
  SensorSample<float> measured_tof;
  MotorOutput output;
  RaceStatus status;
  std::array<SensorObservation, kIrSensorCount> ir;
  std::array<SensorObservation, kTofSensorCount> tof;
  VerdictKind verdict = VerdictKind::kPending;
};

inline constexpr std::uint32_t kMaxTraceFrames = 180001;

}  // namespace lf::sim
