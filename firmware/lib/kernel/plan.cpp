#include "kernel/plan.hpp"

#include <algorithm>
#include <cmath>

namespace lf::kernel {
namespace {

// Binarize giá trị trung bình trượt của obstacle_side về ±1/0.
float binarize_side(float mean) {
  constexpr float kThreshold = 0.1f;
  if (mean > kThreshold) {
    return 1.0f;
  }
  if (mean < -kThreshold) {
    return -1.0f;
  }
  return 0.0f;
}

// Làm mượt obstacle_side bằng trung bình trượt (chịu trôi odometry).
void smooth_obstacle_side(TrackPlan& plan, const TuneConfig& cfg) {
  const std::size_t count = plan.steer_ff.size();
  std::vector<float> smoothed(count, 0.0f);
  const int window = std::max(1, cfg.obstacle_window);
  for (std::size_t i = 0; i < count; ++i) {
    float sum = 0.0f;
    int samples = 0;
    for (int offset = -window; offset <= window; ++offset) {
      const auto j = static_cast<long>(i) + offset;
      if (j >= 0 && j < static_cast<long>(count)) {
        sum += plan.obstacle_side[j];
        ++samples;
      }
    }
    const float mean = (samples > 0) ? (sum / static_cast<float>(samples)) : 0.0f;
    smoothed[i] = binarize_side(mean);
  }
  plan.obstacle_side.swap(smoothed);
}

// Backward-pass tick-space: giảm ga trước khúc cua (ràng buộc phanh phi-cục bộ).
void bake_throttle(TrackPlan& plan, const TuneConfig& cfg) {
  const std::size_t count = plan.steer_ff.size();
  plan.throttle_max.assign(count, 0.0f);
  float next = 1.0f;
  for (std::size_t i = count; i > 0; --i) {
    const std::size_t index = i - 1;
    const float grip = 1.0f - cfg.corner_slowdown * std::abs(plan.steer_ff[index]);
    const float capped = std::min(grip, next + cfg.decel_per_tick);
    plan.throttle_max[index] = std::max(0.0f, capped);
    next = plan.throttle_max[index];
  }
}

}  // namespace

void record(TrackPlan& plan, float tick, float steer, float obstacle_side) {
  if (tick < 0.0f) {
    return;
  }
  const auto index = static_cast<std::size_t>(std::lround(tick));
  if (index >= plan.steer_ff.size()) {
    const float prev_steer = plan.steer_ff.empty() ? 0.0f : plan.steer_ff.back();
    const float prev_side = plan.obstacle_side.empty() ? 0.0f : plan.obstacle_side.back();
    plan.steer_ff.resize(index + 1, prev_steer);
    plan.obstacle_side.resize(index + 1, prev_side);
  }
  plan.steer_ff[index] = steer;
  plan.obstacle_side[index] = obstacle_side;
}

float lookup(const std::vector<float>& values, float tick) {
  if (values.empty()) {
    return 0.0f;
  }
  if (tick <= 0.0f) {
    return values.front();
  }
  const auto last = static_cast<float>(values.size() - 1);
  if (tick >= last) {
    return values.back();
  }
  const auto index = static_cast<std::size_t>(tick);
  const float frac = tick - static_cast<float>(index);
  return values[index] * (1.0f - frac) + values[index + 1] * frac;
}

void bake(TrackPlan& plan, const TuneConfig& cfg) {
  if (plan.steer_ff.empty()) {
    return;
  }
  smooth_obstacle_side(plan, cfg);
  bake_throttle(plan, cfg);
}

}  // namespace lf::kernel
