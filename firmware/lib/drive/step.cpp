#include "drive/step.hpp"

#include <algorithm>
#include <cmath>

namespace lf::drive {
namespace {

constexpr float kMinSpeed = 0.05f;         // tick/s — chặn chia 0 khi tính gain
constexpr float kCertaintyFloor = 0.05f;   // nền tin cậy cho trọng số liên tục
constexpr float kLostSpeed = 0.25f;        // tốc độ tìm vạch khi mất line (không dừng hẳn)
constexpr float kRun2FeedbackGain = 0.4f;  // Run 2 = feedforward-dominant → PD nhẹ

// Né vật cản (ticket 36): ngưỡng suy từ hình học ToF + state machine side-commit.
constexpr float kDangerTrigger = 0.5f;  // danger > 0.5 ⟺ d < kTofRange/2
constexpr float kClearTrigger = 0.2f;   // đã qua vật cản
constexpr float kSideEps = 0.15f;       // |obstacle_side| dưới ngưỡng = chính diện
constexpr float kDefaultSteer = 1.0f;   // chính diện → mặc định vòng phải
constexpr float kAvoidSteer = 0.6f;     // lệch lái vòng (geometry: obstacle_half+robot_half)
constexpr float kBrakeSpeed = 2.0f;     // tick/s — đủ chậm mới cam kết
constexpr float kAvoidSpeed = 0.3f;   // tốc độ khi vòng (đủ chậm để tránh, đủ nhanh để lái vi sai)
constexpr float kRejoinSpeed = 0.6f;  // tốc độ khi bám lại line
constexpr float kRejoinEps = 0.5f;    // bám lại line khi |e| nhỏ

// Hệ số lọc 1st-order từ bandwidth: alpha = dt/(dt + 1/ωn).
float filter_alpha(float bandwidth, float dt) {
  const float tau = 1.0f / std::max(bandwidth, 1e-3f);
  return dt / (dt + tau);
}

// Trọng số tin cậy vạch: w→1 khi rõ vạch, w→0 khi mất vạch.
float certainty_weight(float certainty) {
  return certainty / (certainty + kCertaintyFloor);
}

// ψ ước lượng = đạo hàm lệch ngang theo quãng đường (de/ds ≈ ψ), lọc theo bandwidth.
// gyro (f.heading_rate) KHÔNG dùng trực tiếp: ω là yaw tuyệt đối, không quy chiếu về
// tiếp tuyến line khi chưa biết κ → sẽ thành positive feedback. Gyro dành cho slip
// detection + calibration (ticket 34).
float estimate_heading(const Readings& /*f*/, DriveState& s, const Tuning& cfg, float dt) {
  const float v = std::max(s.speed, kMinSpeed);
  const float offset_rate = (s.line_estimate - s.prev_line_estimate) / std::max(dt, 1e-4f);
  const float psi_offset = cfg.robot.geometry_s * offset_rate / v;
  s.heading += filter_alpha(cfg.robot.bandwidth, dt) * (psi_offset - s.heading);
  return s.heading;
}

// Run 1: ước lượng κ từ steer (u_ss = L·κ) → tra envelope đo (ticket 34/35).
// Run 2: phát lại speed_limit đã bake (backward-pass).
void update_run1_speed(DriveState& s, float target, const Tuning& cfg, float dt) {
  const float alpha = filter_alpha(cfg.robot.bandwidth * 3.0f, dt);
  s.run1_speed += alpha * (target - s.run1_speed);
  s.run1_speed = std::clamp(s.run1_speed, 0.0f, 1.0f);
}

struct AvoidBias {
  float bias = 0.0f;         // lệch lái
  float speed_scale = 1.0f;  // nhân tốc độ
  bool brake = false;
};

// Vòng state machine né vật cản (ticket 36): phanh → cam kết bên → vòng → bám lại.
AvoidBias avoid_steer(const Readings& f, DriveState& s, const RacePlan& plan, float tick) {
  AvoidBias out;
  switch (s.avoid) {
    case AvoidPhase::kIdle:
      if (f.obstacle_danger > kDangerTrigger) {
        // avoid_side = HƯỚNG LÁI (tránh xa vật cản); chính diện → mặc định phải.
        s.avoid_side = (std::abs(f.obstacle_side) > kSideEps)
                         ? -std::copysign(1.0f, f.obstacle_side)
                         : kDefaultSteer;
        s.avoid = AvoidPhase::kBrake;
      }
      break;
    case AvoidPhase::kBrake:
      out.brake = true;
      if (s.speed < kBrakeSpeed) {
        s.avoid = AvoidPhase::kCommit;
      }
      break;
    case AvoidPhase::kCommit:
      out.bias = s.avoid_side * kAvoidSteer;
      out.speed_scale = kAvoidSpeed;
      if (f.obstacle_danger < kClearTrigger) {
        s.avoid = AvoidPhase::kRejoin;
      }
      break;
    case AvoidPhase::kRejoin:
      out.bias = s.avoid_side * kAvoidSteer * 0.5f;
      out.speed_scale = kRejoinSpeed;
      if (std::abs(f.line_offset) < kRejoinEps && f.line_certainty > 0.5f) {
        s.avoid = AvoidPhase::kIdle;
      }
      break;
  }
  // Run 2 biết trước bên né (ghi ở Run 1) → feedforward sớm; plan chưa ready trả 0.
  out.bias += plan.obstacle_at(tick) * kAvoidSteer;
  return out;
}

}

// Stage 1 — chuyển động: encoder → quãng đường tick + tốc độ lọc.
MotionEstimate estimate_motion(const Readings& frame, const DriveState& state, const Tuning& cfg,
                               float dt) {
  // Encoder thô ở 20mm/tick nên cần lọc mạnh.
  const float tick = 0.5f * (static_cast<float>(frame.enc_l) + static_cast<float>(frame.enc_r));
  const float speed_raw = (tick - state.prev_tick) / dt;
  const float alpha = filter_alpha(cfg.robot.bandwidth * 0.5f, dt);
  return {tick, state.speed + alpha * (speed_raw - state.speed)};
}

// Stage 2 — niềm tin vạch: sensor + ước lượng → lệch (đã trọng số) + heading.
// Mất vạch (certainty → 0) → line_estimate đứng yên (đi thẳng mượt), không slam ±1.
LineBelief estimate_line(const Readings& frame, DriveState& next, const RacePlan& plan, float tick,
                         float dt, const Tuning& cfg) {
  LineBelief belief;
  belief.certainty = certainty_weight(frame.line_certainty);
  const float estimate = fuse_line_offset(frame, next);
  // plan chưa ready → offset_at trả 0 (không cần rẽ nhánh).
  belief.offset = belief.certainty * (estimate - plan.offset_at(tick));
  belief.heading = estimate_heading(frame, next, cfg, dt);
  return belief;
}

// Niềm tin liên tục về lệch vạch: đo + ước lượng, trọng số theo độ tin cậy sensor.
float fuse_line_offset(const Readings& frame, DriveState& state) {
  const float blend = certainty_weight(frame.line_certainty);
  state.line_estimate += blend * (frame.line_offset - state.line_estimate);
  return state.line_estimate;
}

// Pole-placement ζ cố định (ticket 33): plant ë = (v²/(L·s))·u → kp/kd schedule theo v.
//   kp    = ωn²·L·s / v²
//   k_psi = 2·ζ·ωn·L / v        (damping theo heading ψ thay vì đạo hàm offset)
float steer_to_line(float error, float heading, float v_tick, DriveState& /*state*/,
                    const Tuning& cfg) {
  const float v = std::max(v_tick, kMinSpeed);
  const float omega2 = cfg.robot.bandwidth * cfg.robot.bandwidth;
  const float ls = cfg.robot.geometry_l * cfg.robot.geometry_s;
  const float kp = omega2 * ls / (v * v);
  const float k_psi = 2.0f * cfg.robot.damping * cfg.robot.bandwidth * cfg.robot.geometry_l / v;
  return std::clamp(kp * error + k_psi * heading, -1.0f, 1.0f);
}

float choose_speed(float tick, float error, float steer, const Readings& f, const RacePlan& plan,
                   DriveState& s, const Tuning& cfg) {
  float target = 0.0f;
  if (plan.ready) {
    target = cfg.track.sprint_margin * plan.speed_limit_at(tick);  // Run 2: envelope đã bake
  } else {
    const float curve_limit = cfg.robot.speed_limit(std::abs(steer));
    // Ngân sách sai số (phản ứng nhanh hơn κ-theo-steer): |e|→1 thì về sàn bò.
    const float budget = std::clamp(1.0f - std::abs(error), kLostSpeed, 1.0f);
    target = std::min(curve_limit, budget);
  }
  // Mất vạch (weight→0) → bò chậm tìm vạch (không dừng hẳn để kịp re-find < 5s).
  const float weight = std::max(certainty_weight(f.line_certainty), kLostSpeed);
  update_run1_speed(s, weight * target, cfg, f.dt);
  return s.run1_speed;
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

// Pass-manager: gọi các stage theo thứ tự, không chứa logic điều khiển nội tuyến.
StepResult step(const DriveState& state, const Readings& frame, const RacePlan& plan,
                const Tuning& cfg) {
  DriveState next = state;
  const float dt = std::max(frame.dt, 1e-4f);

  const MotionEstimate motion = estimate_motion(frame, state, cfg, dt);
  next.prev_tick = motion.tick;
  next.speed = motion.speed;

  next.prev_line_estimate = state.line_estimate;
  const LineBelief belief = estimate_line(frame, next, plan, motion.tick, dt, cfg);

  // Stage 3 — lái: feedforward (Run 2) + feedback; Run 1 thuần phản ứng.
  const float feedback_gain = plan.ready ? kRun2FeedbackGain : 1.0f;
  const float path_steer =
    plan.line_curve_at(motion.tick) +
    feedback_gain * steer_to_line(belief.offset, belief.heading, next.speed, next, cfg);

  // Stage 4 — né vật cản (bias + tốc độ).
  const AvoidBias avoid = avoid_steer(frame, next, plan, motion.tick);

  // Stage 5 — tốc độ.
  const float steer = std::clamp(path_steer + avoid.bias, -1.0f, 1.0f);
  const float throttle =
    choose_speed(motion.tick, belief.offset, steer, frame, plan, next, cfg) * avoid.speed_scale;

  // Stage 6 — trộn ra lệnh bánh.
  next.finish_latched = state.finish_latched || frame.finish_detected;
  const DriveCommand command = drive_motors(throttle, steer, next.finish_latched || avoid.brake);
  return {command, next, path_steer};
}

}
