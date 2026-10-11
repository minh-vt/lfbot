#include "drive/step.hpp"

#include <algorithm>
#include <cmath>

namespace lf::drive {
namespace {

// Né vật cản: Run 1 phản ứng (ToF); Run 2 feedforward (plan đã plan_speed) + ToF safety trim.
float avoid_obstacle(const Readings& frame, const RacePlan& plan, float tick, const Tuning& cfg) {
  const float reactive = avoid_strength(frame, cfg);
  if (!plan.ready) {
    return reactive;
  }
  constexpr float kSafetyTrim = 0.25f;
  return -plan.obstacle_at(tick) * cfg.robot.avoid_strength + kSafetyTrim * reactive;
}

}

// Niềm tin liên tục về lệch vạch: đo + ước lượng, trọng số theo độ tin cậy sensor.
// Mất vạch (certainty → 0) → line_estimate đứng yên (đi thẳng mượt), không slam ±1.
float fuse_line_offset(const Readings& frame, DriveState& state) {
  constexpr float kCertaintyFloor = 0.05f;
  const float blend = frame.line_certainty / (frame.line_certainty + kCertaintyFloor);
  state.line_estimate += blend * (frame.line_offset - state.line_estimate);
  return state.line_estimate;
}

float steer_to_line(float error, float dt, DriveState& state, const Tuning& cfg) {
  const float safe_dt = std::max(dt, 1e-4f);
  const float derivative = (error - state.prev_error) / safe_dt;
  state.prev_error = error;
  state.filtered_derivative = cfg.robot.d_filter_alpha * derivative +
                              (1.0f - cfg.robot.d_filter_alpha) * state.filtered_derivative;
  return std::clamp(cfg.robot.steer_kp * error + cfg.robot.steer_kd * state.filtered_derivative,
                    -1.0f, 1.0f);
}

float avoid_strength(const Readings& frame, const Tuning& cfg) {
  // obstacle_side < 0 = vật cản bên trái → rẽ phải (+); ngược lại rẽ trái (−).
  return -frame.obstacle_side * cfg.robot.avoid_strength * frame.obstacle_danger;
}

float choose_speed(float tick, float steer, const Readings& frame, const RacePlan& plan,
                   const Tuning& cfg) {
  // Run 2 phát lại speed_limit đã bake; Run 1 giảm ga theo giới hạn tốc độ ở steer hiện tại.
  const float base = plan.ready ? cfg.track.sprint_scale * plan.speed_limit_at(tick)
                                : cfg.track.base_throttle * cfg.robot.speed_limit(steer);
  const float throttle = base * (1.0f - cfg.robot.avoid_slowdown * frame.obstacle_danger);
  return std::clamp(throttle, 0.0f, 1.0f);
}

DriveCommand drive_motors(float throttle, float steer, bool brake) {
  const float bounded_throttle = std::clamp(throttle, 0.0f, 1.0f);
  const float bounded_steer = std::clamp(steer, -1.0f, 1.0f);
  if (brake) {
    return {0.0f, 0.0f, true};
  }
  // steer > 0 = rẽ phải (CW): bánh phải nhanh hơn.
  float left = bounded_throttle * (1.0f - bounded_steer);
  float right = bounded_throttle * (1.0f + bounded_steer);
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

  // Arc-length index (tick) = trung bình encoder.
  const float tick = 0.5f * (static_cast<float>(frame.enc_l) + static_cast<float>(frame.enc_r));

  // Setpoint lệch ngang (racing line) — Run 2 mới có; Run 1 theo tâm vạch.
  const float setpoint = plan.ready ? plan.offset_at(tick) : 0.0f;
  const float error = fuse_line_offset(frame, next) - setpoint;

  // Feedforward (geometry) CHỈ khi plan ready (Run 2); Run 1 thuần phản ứng.
  const float feedforward = plan.ready ? plan.line_curve_at(tick) : 0.0f;
  const float path_steer = feedforward + steer_to_line(error, frame.dt, next, cfg);
  const float steer = std::clamp(path_steer + avoid_obstacle(frame, plan, tick, cfg), -1.0f, 1.0f);
  const float throttle = choose_speed(tick, steer, frame, plan, cfg);

  next.finish_latched = state.finish_latched || frame.finish_detected;
  const DriveCommand command = drive_motors(throttle, steer, next.finish_latched);
  return {command, next, path_steer};
}

}
