#include "drive/step.hpp"

#include <algorithm>
#include <cmath>

namespace lf::drive {
namespace {

constexpr float kMinSpeed = 0.05f;
constexpr float kCertaintyFloor = 0.05f;
constexpr float kLostSpeed = 0.25f;

constexpr float kSideEps = 0.15f;
constexpr float kRepelSteer = 1.0f;
constexpr float kAvoidSlow = 0.85f;
constexpr float kSideMemory = 0.3f;
constexpr float kTieBreakSide = 1.0f;

float filter_alpha(float bandwidth, float dt) {
  return dt / (dt + 1.0f / std::max(bandwidth, 1e-3f));
}

float certainty_weight(float certainty) {
  return certainty / (certainty + kCertaintyFloor);
}

float update_heading(DriveState& s, float error_rate, const Tuning& cfg, float dt) {
  const float v = std::max(s.speed, kMinSpeed);
  const float psi = cfg.robot.geometry_s * error_rate / v;
  s.heading += filter_alpha(cfg.robot.bandwidth, dt) * (psi - s.heading);
  return s.heading;
}

void update_run1_speed(DriveState& s, float target, const Tuning& cfg, float dt) {
  const float alpha = filter_alpha(cfg.robot.bandwidth * 3.0f, dt);
  s.run1_speed += alpha * (target - s.run1_speed);
  s.run1_speed = std::clamp(s.run1_speed, 0.0f, 1.0f);
}

}  // namespace

MotionEstimate estimate_motion(const Readings& frame, const DriveState& state, const Tuning& cfg,
                               float dt) {
  const float tick = 0.5f * (static_cast<float>(frame.enc_l) + static_cast<float>(frame.enc_r));
  const float speed_raw = (tick - state.prev_tick) / dt;
  const float alpha = filter_alpha(cfg.robot.bandwidth * 0.5f, dt);
  return {tick, state.speed + alpha * (speed_raw - state.speed)};
}

LineBelief estimate_line(const Readings& frame, DriveState& next, const RacePlan& plan, float tick,
                         float dt, const Tuning& cfg) {
  LineBelief belief;
  belief.certainty = certainty_weight(frame.line_certainty);
  const float estimate = fuse_line_offset(frame, next);
  const float error = estimate - plan.lateral_at(tick);
  belief.offset = belief.certainty * error;
  const float error_rate = (error - next.prev_error) / std::max(dt, 1e-4f);
  next.prev_error = error;
  belief.heading = update_heading(next, error_rate, cfg, dt);
  return belief;
}

float fuse_line_offset(const Readings& frame, DriveState& state) {
  const float blend = certainty_weight(frame.line_certainty);
  state.line_estimate += blend * (frame.line_offset - state.line_estimate);
  return state.line_estimate;
}

float steer_to_line(float error, float heading) {
  constexpr float kp = 1.0f;
  constexpr float kd = 0.5f;
  return std::clamp(kp * error + kd * heading, -1.0f, 1.0f);
}

FieldOut apply_field(float path_steer, const Readings& f, DriveState& s) {
  const float w = f.obstacle_danger;
  FieldOut out;
  out.speed_scale = 1.0f - w * kAvoidSlow;
  if (w <= 0.0f) {
    out.steer = path_steer;
    return out;
  }
  const float measured = -f.obstacle_side;
  const float desired = (std::abs(measured) > kSideEps) ? measured : kTieBreakSide;
  s.avoid_side += kSideMemory * (desired - s.avoid_side);
  out.steer = (1.0f - w) * path_steer + w * s.avoid_side * kRepelSteer;
  return out;
}

float choose_speed(float tick, float error, float steer, const Readings& f, const RacePlan& plan,
                   DriveState& s, const Tuning& cfg) {
  float target = 0.0f;
  if (plan.ready) {
    target = plan.speed_limit_at(tick);
  } else {
    const float curve_limit = cfg.robot.speed_limit(std::abs(steer));
    const float budget = std::clamp(1.0f - std::abs(error), kLostSpeed, 1.0f);
    target = std::min(curve_limit, budget);
  }
  const float weight = std::max(certainty_weight(f.line_certainty), kLostSpeed);
  update_run1_speed(s, weight * target, cfg, f.dt);
  return s.run1_speed;
}

DriveCommand drive_motors(float throttle, float steer, bool brake) {
  const float t = std::clamp(throttle, 0.0f, 1.0f);
  const float st = std::clamp(steer, -1.0f, 1.0f);
  if (brake) {
    return {0.0f, 0.0f, true};
  }
  float left = t * (1.0f - st);
  float right = t * (1.0f + st);
  const float peak = std::max(std::abs(left), std::abs(right));
  if (peak > 1.0f) {
    left /= peak;
    right /= peak;
  }
  return {left, right, false};
}

StepResult step(const DriveState& state, const Readings& frame, const RacePlan& plan,
                const Tuning& cfg) {
  DriveState next = state;
  const float dt = std::max(frame.dt, 1e-4f);

  const MotionEstimate motion = estimate_motion(frame, state, cfg, dt);
  next.prev_tick = motion.tick;
  next.speed = motion.speed;

  const LineBelief belief = estimate_line(frame, next, plan, motion.tick, dt, cfg);

  const float path_steer = steer_to_line(belief.offset, belief.heading);
  const FieldOut field = apply_field(path_steer, frame, next);

  const float steer = std::clamp(field.steer, -1.0f, 1.0f);
  const float throttle =
    choose_speed(motion.tick, belief.offset, steer, frame, plan, next, cfg) * field.speed_scale;

  next.finish_latched = state.finish_latched || frame.finish_detected;
  const DriveCommand command = drive_motors(throttle, steer, next.finish_latched);
  return {command, next, path_steer};
}

}
