#pragma once

#include <cstdint>

#include "kernel/plan.hpp"
#include "kernel/step.hpp"

namespace lf::kernel {

// tick = quãng đường trung bình 2 bánh (đơn vị: tick encoder).
inline float tick_from_encoders(const SensorFrame& frame) {
  return 0.5f * (static_cast<float>(frame.enc_l) + static_cast<float>(frame.enc_r));
}

// Một tick trí tuệ: step + ghi plan ở Run 1. Trả command để shell ghi ra adapter.
inline MotorCommand mission_step(KernelState& state, const SensorFrame& frame, TrackPlan& plan,
                                 RunMode mode, const TuneConfig& cfg) {
  const StepResult result = step(state, frame, plan, cfg);
  state = result.state;
  if (mode == RunMode::kRun1) {
    record(plan, tick_from_encoders(frame), result.path_steer, frame.obstacle_side);
  }
  return result.command;
}

// Sự kiện mission phát sinh sau một tick.
enum class MissionEvent : std::uint8_t { kNone, kRun1Done, kRun2Done };

// Chuyển mode khi finish-latched: bake khi hết Run 1, kết thúc ở Run 2.
// Shell tự reset adapter / ghi brake theo sự kiện trả về.
inline MissionEvent advance_mission(TrackPlan& plan, KernelState& state, RunMode& mode,
                                    const TuneConfig& cfg) {
  if (mode == RunMode::kRun1 && state.finish_latched) {
    bake(plan, cfg);
    state = {};
    mode = RunMode::kRun2;
    return MissionEvent::kRun1Done;
  }
  if (mode == RunMode::kRun2 && state.finish_latched) {
    mode = RunMode::kDone;
    return MissionEvent::kRun2Done;
  }
  return MissionEvent::kNone;
}

}
