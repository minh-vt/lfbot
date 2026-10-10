#pragma once

#include "kernel/types.hpp"

namespace lf::kernel {

struct StepResult {
  MotorCommand command;
  KernelState state;
  float path_steer = 0.0f;  // steer_ff + PD(e): steering theo vạch (không né vật cản)
};

// SEAM NGOÀI — thuần, mode-agnostic (plan rỗng = Run 1, plan đầy = Run 2).
StepResult step(const KernelState& s, const SensorFrame& f, const TrackPlan& plan,
                const TuneConfig& cfg);

// Internal seams (thuần, test trực tiếp).
float resolve_line_error(const SensorFrame& f, const KernelState& s);
float steer_pd(float error, float dt, KernelState& s, const TuneConfig& cfg);
float evasion_bias(const SensorFrame& f, const TuneConfig& cfg);
float throttle_profile(float tick, float steer, const SensorFrame& f, const TrackPlan& plan,
                       const TuneConfig& cfg);
MotorCommand mix_differential(float throttle, float steer, bool brake);

}  // namespace lf::kernel
