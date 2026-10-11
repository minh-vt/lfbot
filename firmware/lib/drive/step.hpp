#pragma once

#include "drive/types.hpp"

namespace lf::drive {

struct StepResult {
  DriveCommand command;
  DriveState state;
  float path_steer = 0.0f;  // planned_steer + PD(e): steering theo vạch (không né vật cản)
};

// SEAM NGOÀI — thuần, mode-agnostic (plan rỗng = Run 1, plan đầy = Run 2).
StepResult step(const DriveState& s, const Readings& f, const RacePlan& plan, const Tuning& cfg);

// Internal seams (thuần, test trực tiếp).
float fuse_line_offset(const Readings& f, DriveState& s);
float steer_to_line(float error, float dt, DriveState& s, const Tuning& cfg);
float avoid_strength(const Readings& f, const Tuning& cfg);
float choose_speed(float tick, float steer, const Readings& f, const RacePlan& plan,
                   const Tuning& cfg);
DriveCommand drive_motors(float throttle, float steer, bool brake);

}
