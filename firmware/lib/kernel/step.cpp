#include "kernel/step.hpp"

#include <algorithm>
#include <cmath>

namespace lf::kernel {
namespace {

// Né vật cản: Run 1 phản ứng (ToF); Run 2 feedforward (plan đã bake) + ToF safety trim.
float steer_evasion(const SensorFrame& frame, const TrackPlan& plan, float tick,
                    const TuneConfig& cfg) {
  const float reactive = evasion_bias(frame, cfg);
  if (!plan.baked) {
    return reactive;
  }
  constexpr float kSafetyTrim = 0.25f;
  return -plan.obstacle_at(tick) * cfg.evasion_bias + kSafetyTrim * reactive;
}

}  // namespace

float resolve_line_error(const SensorFrame& frame, const KernelState& state) {
  if (frame.line_detected) {
    return std::clamp(frame.line_error, -1.0f, 1.0f);
  }
  return state.last_direction;  // ±1: tiếp tục rẽ về phía vạch nhớ
}

float steer_pd(float error, float dt, KernelState& state, const TuneConfig& cfg) {
  const float safe_dt = std::max(dt, 1e-4f);
  const float derivative = (error - state.prev_error) / safe_dt;
  state.prev_error = error;
  state.filtered_derivative =
    cfg.d_filter_alpha * derivative + (1.0f - cfg.d_filter_alpha) * state.filtered_derivative;
  return std::clamp(cfg.steer_kp * error + cfg.steer_kd * state.filtered_derivative, -1.0f, 1.0f);
}

float evasion_bias(const SensorFrame& frame, const TuneConfig& cfg) {
  // obstacle_side < 0 = vật cản bên trái → rẽ phải (+); ngược lại rẽ trái (−).
  return -frame.obstacle_side * cfg.evasion_bias * frame.obstacle_danger;
}

float throttle_profile(float tick, float steer, const SensorFrame& frame, const TrackPlan& plan,
                       const TuneConfig& cfg) {
  float throttle = 0.0f;
  if (plan.baked) {
    // Run 2 phát lại: profile đã bake.
    throttle = cfg.sprint_scale * plan.throttle_at(tick);
  } else {
    // Run 1 phản ứng: giảm ga theo độ gắt lái hiện tại.
    throttle = cfg.base_throttle * (1.0f - cfg.corner_slowdown * std::abs(steer));
  }
  throttle *= (1.0f - cfg.evasion_slowdown * frame.obstacle_danger);
  return std::clamp(throttle, 0.0f, 1.0f);
}

MotorCommand mix_differential(float throttle, float steer, bool brake) {
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
  return {std::clamp(left, -1.0f, 1.0f), std::clamp(right, -1.0f, 1.0f), false};
}

StepResult step(const KernelState& state, const SensorFrame& frame, const TrackPlan& plan,
                const TuneConfig& cfg) {
  KernelState next = state;

  // Arc-length index (tick) = trung bình encoder.
  const float tick = 0.5f * (static_cast<float>(frame.enc_l) + static_cast<float>(frame.enc_r));

  const float error = resolve_line_error(frame, state);
  if (frame.line_detected) {
    next.last_direction = (frame.line_error >= 0.0f) ? 1.0f : -1.0f;
  }

  // Feedforward CHỈ khi plan đã bake (Run 2); Run 1 thuần phản ứng (tránh self-feedforward).
  const float steer_ff = plan.baked ? plan.steer_at(tick) : 0.0f;
  const float steer_fb = steer_pd(error, frame.dt, next, cfg);
  const float path_steer = steer_ff + steer_fb;
  const float steer = std::clamp(path_steer + steer_evasion(frame, plan, tick, cfg), -1.0f, 1.0f);
  const float throttle = throttle_profile(tick, steer, frame, plan, cfg);

  next.finish_latched = state.finish_latched || frame.finish_detected;
  const MotorCommand command = mix_differential(throttle, steer, next.finish_latched);
  return {command, next, path_steer};
}

}  // namespace lf::kernel
