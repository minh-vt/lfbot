#include "control/relay_tuner.hpp"

#include <algorithm>
#include <cmath>

namespace lf {

void RelayAutoTuner::start(float relay_amplitude, int target_cycles) {
  relay_d_ = relay_amplitude;
  target_cycles_ = target_cycles;
  state_ = State::kOscillating;
  cycle_count_ = 0;
  elapsed_time_ = 0.0f;
  peak_amp_ = 0.0f;
  last_zero_cross_time_ = 0.0f;
  prev_error_ = 0.0f;
  total_period_sum_ = 0.0f;
}

float RelayAutoTuner::update(float error, float dt) {
  // Guard clause: Chưa ở chế độ tự dò
  if (state_ != State::kOscillating) {
    return 0.0f;
  }

  elapsed_time_ += dt;

  const float abs_err = std::abs(error);
  peak_amp_ = std::max(abs_err, peak_amp_);

  // Phát hiện giao cắt điểm 0 (Zero-crossing)
  const bool is_zero_crossing =
    (prev_error_ <= 0.0f && error > 0.0f) || (prev_error_ >= 0.0f && error < 0.0f);

  if (is_zero_crossing) {
    if (last_zero_cross_time_ > 0.0f) {
      const float half_period = elapsed_time_ - last_zero_cross_time_;
      total_period_sum_ += half_period * 2.0f;
      cycle_count_++;

      if (cycle_count_ >= target_cycles_) {
        finish_tuning();
      }
    }
    last_zero_cross_time_ = elapsed_time_;
  }

  prev_error_ = error;

  // Xuất tín hiệu Relay
  return (error >= 0.0f) ? relay_d_ : -relay_d_;
}

void RelayAutoTuner::finish_tuning() {
  if (peak_amp_ <= 0.001f || cycle_count_ == 0) {
    state_ = State::kFailed;
    return;
  }

  state_ = State::kConverged;
}

void RelayAutoTuner::get_tuned_gains(float& out_kp, float& out_ki, float& out_kd) const {
  // Guard clause: Nếu chưa hội tụ
  if (state_ != State::kConverged || peak_amp_ <= 0.0001f || cycle_count_ == 0) {
    out_kp = 0.0f;
    out_ki = 0.0f;
    out_kd = 0.0f;
    return;
  }

  constexpr float kPi = 3.14159265f;
  const float ku = (4.0f * relay_d_) / (kPi * peak_amp_);
  const float tu = total_period_sum_ / static_cast<float>(cycle_count_);

  // Quy chuẩn tối ưu cho Robot bám vạch tốc độ cao (High-Response PD + Low-I)
  out_kp = std::clamp(ku * 0.45f, 0.015f, 0.120f);
  out_ki = std::clamp(out_kp / (15.0f * tu), 0.0001f, 0.005f);
  out_kd = std::clamp(out_kp * (tu * 0.12f), 0.002f, 0.035f);
}

}  // namespace lf
