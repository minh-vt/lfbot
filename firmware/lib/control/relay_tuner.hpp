#pragma once

#include <cmath>
#include <cstdint>

namespace lf {

struct TunerResult {
  float kp = 0.0f;
  float kd = 0.0f;
  float ultimate_gain = 0.0f;
  float ultimate_period_s = 0.0f;
  float oscillation_amp_mm = 0.0f;
  int cycles_completed = 0;
  bool converged = false;
};

class RelayTuner {
public:
  explicit RelayTuner(float relay_amplitude = 0.12f, float hysteresis_mm = 1.0f);

  void reset();

  // Updates the tuner with current centroid error (in mm) and timestep dt (in seconds).
  // Returns steering command in m/s (differential wheel velocity).
  float update(float centroid_mm, float dt);

  [[nodiscard]] bool is_converged() const { return result_.converged; }
  [[nodiscard]] const TunerResult& get_result() const { return result_; }
  [[nodiscard]] int get_cycle_count() const { return cycle_count_; }
  [[nodiscard]] float get_current_amplitude_mm() const { return current_amp_mm_; }
  [[nodiscard]] float get_relay_amplitude() const { return relay_amp_; }

private:
  void record_cycle(float period_s, float amplitude_mm);

  float relay_amp_ = 0.12f;
  float hysteresis_mm_ = 1.0f;
  float current_steer_ = 0.0f;

  float peak_pos_mm_ = 0.0f;
  float peak_neg_mm_ = 0.0f;
  float cycle_timer_s_ = 0.0f;
  int cycle_count_ = 0;

  float sum_period_s_ = 0.0f;
  float sum_amplitude_mm_ = 0.0f;
  int valid_cycles_count_ = 0;

  float prev_centroid_mm_ = 0.0f;
  float current_amp_mm_ = 0.0f;

  TunerResult result_{};

  static constexpr int kWarmupCycles = 1;
  static constexpr int kRequiredCycles = 3;
};

}  // namespace lf
