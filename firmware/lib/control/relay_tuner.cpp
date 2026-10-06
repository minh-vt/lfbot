#include "control/relay_tuner.hpp"

#include <algorithm>
#include <cmath>

#include "config/constants.hpp"

namespace lf {

RelayTuner::RelayTuner(float relay_amplitude, float hysteresis_mm)
  : relay_amp_((relay_amplitude > 0.01f) ? relay_amplitude : 0.12f),
    hysteresis_mm_((hysteresis_mm > 0.1f) ? hysteresis_mm : 1.0f) {
  reset();
}

void RelayTuner::reset() {
  current_steer_ = 0.0f;
  peak_pos_mm_ = 0.0f;
  peak_neg_mm_ = 0.0f;
  cycle_timer_s_ = 0.0f;
  cycle_count_ = 0;
  sum_period_s_ = 0.0f;
  sum_amplitude_mm_ = 0.0f;
  valid_cycles_count_ = 0;
  prev_centroid_mm_ = 0.0f;
  current_amp_mm_ = 0.0f;
  result_ = TunerResult{};
}

float RelayTuner::update(float centroid_mm, float dt) {
  if (result_.converged || dt <= 0.00001f) {
    return 0.0f;
  }

  // 1. Relay switching with deadband hysteresis
  if (centroid_mm > hysteresis_mm_) {
    current_steer_ = relay_amp_;
  } else if (centroid_mm < -hysteresis_mm_) {
    current_steer_ = -relay_amp_;
  } else if (std::abs(current_steer_) < 0.001f) {
    current_steer_ = (centroid_mm >= 0.0f) ? relay_amp_ : -relay_amp_;
  }

  // 2. Track peaks within current cycle
  cycle_timer_s_ += dt;
  peak_pos_mm_ = std::max(peak_pos_mm_, centroid_mm);
  peak_neg_mm_ = std::min(peak_neg_mm_, centroid_mm);

  // 3. Detect rising zero-crossing (negative to positive transition)
  const bool rising_crossing = (prev_centroid_mm_ <= 0.0f && centroid_mm > 0.0f);
  if (rising_crossing && cycle_timer_s_ > 0.12f) {
    const float cycle_period = cycle_timer_s_;
    const float cycle_amp = 0.5f * (peak_pos_mm_ - peak_neg_mm_);

    record_cycle(cycle_period, cycle_amp);

    peak_pos_mm_ = centroid_mm;
    peak_neg_mm_ = 0.0f;
    cycle_timer_s_ = 0.0f;
  }

  prev_centroid_mm_ = centroid_mm;
  return current_steer_;
}

void RelayTuner::record_cycle(float period_s, float amplitude_mm) {
  ++cycle_count_;
  if (cycle_count_ <= kWarmupCycles) {
    return;
  }

  sum_period_s_ += period_s;
  sum_amplitude_mm_ += amplitude_mm;
  ++valid_cycles_count_;
  current_amp_mm_ = amplitude_mm;

  if (valid_cycles_count_ < kRequiredCycles) {
    return;
  }

  const float avg_tu = sum_period_s_ / static_cast<float>(valid_cycles_count_);
  const float avg_amp = sum_amplitude_mm_ / static_cast<float>(valid_cycles_count_);

  if (avg_amp <= 0.5f) {
    return;
  }

  // Åström–Hägglund formula for ultimate gain: Ku = (4 * d) / (pi * A)
  const float ku = (4.0f * relay_amp_) / (constants::kPi * avg_amp);

  // Ziegler-Nichols PD tuning rule
  float kp = 0.60f * ku;
  float kd = kp * (avg_tu / 8.0f);

  // Physical guardrails
  kp = std::clamp(kp, 0.005f, 0.150f);
  kd = std::clamp(kd, 0.0005f, 0.050f);

  result_.kp = kp;
  result_.kd = kd;
  result_.ultimate_gain = ku;
  result_.ultimate_period_s = avg_tu;
  result_.oscillation_amp_mm = avg_amp;
  result_.cycles_completed = cycle_count_;
  result_.converged = true;
}

}  // namespace lf
