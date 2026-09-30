#pragma once

#include <cstdint>

namespace lf {

class RelayAutoTuner {
public:
  enum class State : uint8_t { kIdle, kOscillating, kConverged, kFailed };

  void start(float relay_amplitude, int target_cycles = 5);
  float update(float error, float dt);

  [[nodiscard]] State get_state() const { return state_; }
  void get_tuned_gains(float& out_kp, float& out_ki, float& out_kd) const;

private:
  void finish_tuning();

  State state_ = State::kIdle;
  float relay_d_ = 0.4f;
  int target_cycles_ = 5;
  int cycle_count_ = 0;
  float elapsed_time_ = 0.0f;
  float peak_amp_ = 0.0f;
  float last_zero_cross_time_ = 0.0f;
  float prev_error_ = 0.0f;
  float total_period_sum_ = 0.0f;
};

}  // namespace lf
