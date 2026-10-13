#pragma once

#include "drive/types.hpp"

namespace lf::drive {

struct StepResult {
  DriveCommand command;
  DriveState state;
  float path_steer = 0.0f;  // planned_steer + steer_to_line(e): lái theo vạch (không né vật cản)
};

// IR giữa các stage của pipeline điều khiển (mỗi tick).
struct MotionEstimate {
  float tick = 0.0f;   // quãng đường odometry trung bình
  float speed = 0.0f;  // tick/s, đã lọc
};

struct LineBelief {
  float offset = 0.0f;     // lệch vạch so với setpoint (đã trọng số tin cậy)
  float certainty = 0.0f;  // [0,1] trọng số tin cậy
  float heading = 0.0f;    // ψ ước lượng
};

// SEAM NGOÀI — thuần, mode-agnostic (plan rỗng = Run 1, plan đầy = Run 2).
StepResult step(const DriveState& s, const Readings& f, const RacePlan& plan, const Tuning& cfg);

// Internal seams — các stage của pipeline (thuần, test trực tiếp).
MotionEstimate estimate_motion(const Readings& f, const DriveState& s, const Tuning& cfg, float dt);
LineBelief estimate_line(const Readings& f, DriveState& next, const RacePlan& plan, float tick,
                         float dt, const Tuning& cfg);
float fuse_line_offset(const Readings& f, DriveState& s);
float steer_to_line(float error, float heading, float v_tick, DriveState& s, const Tuning& cfg);
float choose_speed(float tick, float error, float steer, const Readings& f, const RacePlan& plan,
                   DriveState& s, const Tuning& cfg);
DriveCommand drive_motors(float throttle, float steer, bool brake);

}
