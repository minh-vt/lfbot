#pragma once

#include "drive/types.hpp"

namespace lf::drive {

struct StepResult {
  DriveCommand command;
  DriveState state;
  float path_steer =
    0.0f;  // line_curve feedforward + steer_to_line(e): lái theo vạch (không né vật cản)
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

// Kết quả trường thế né vật cản (ticket 03): lái đã trộn + tỉ lệ tốc độ.
struct FieldOut {
  float steer = 0.0f;        // lái sau khi trộn line (hút) ↔ vật cản (đẩy)
  float speed_scale = 1.0f;  // giảm tốc liên tục theo danger
};

// SEAM NGOÀI — thuần, mode-agnostic (plan rỗng = Run 1, plan đầy = Run 2).
StepResult step(const DriveState& s, const Readings& f, const RacePlan& plan, const Tuning& cfg);

// Internal seams — các stage của pipeline (thuần, test trực tiếp).
MotionEstimate estimate_motion(const Readings& f, const DriveState& s, const Tuning& cfg, float dt);
LineBelief estimate_line(const Readings& f, DriveState& next, const RacePlan& plan, float tick,
                         float dt, const Tuning& cfg);
float fuse_line_offset(const Readings& f, DriveState& s);
float steer_to_line(float error, float heading, float v_tick, const Tuning& cfg);
// Trường thế liên tục: trộn lái bám line (hút) với lái né vật cản (đẩy) theo danger.
FieldOut apply_field(float path_steer, const Readings& f, DriveState& s);
float choose_speed(float tick, float error, float steer, const Readings& f, const RacePlan& plan,
                   DriveState& s, const Tuning& cfg);
DriveCommand drive_motors(float throttle, float steer, bool brake);

}
