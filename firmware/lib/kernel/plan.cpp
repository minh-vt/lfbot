#include "kernel/plan.hpp"

#include <algorithm>
#include <cmath>

namespace lf::kernel {
namespace {

// Nội suy tuyến tính trên [0, length); trả 0 nếu rỗng.
float interp(const float* values, std::size_t length, float tick) {
  if (length == 0) {
    return 0.0f;
  }
  if (tick <= 0.0f) {
    return values[0];
  }
  const auto last = static_cast<float>(length - 1);
  if (tick >= last) {
    return values[length - 1];
  }
  const auto index = static_cast<std::size_t>(tick);
  const float frac = tick - static_cast<float>(index);
  return values[index] * (1.0f - frac) + values[index + 1] * frac;
}

// Giãn obstacle_side về phía tick nhỏ hơn (né sớm), in-place, không bộ nhớ tạm.
void dilate_obstacle_side(TrackPlan& plan, const TuneConfig& cfg) {
  const int span = 2 * std::max(1, static_cast<int>(cfg.obstacle_window));
  float side = 0.0f;
  int hold = 0;
  for (std::size_t i = plan.length; i > 0; --i) {
    const std::size_t index = i - 1;
    if (plan.obstacle_side[index] != 0.0f) {
      side = plan.obstacle_side[index];
      hold = span;
    } else if (hold > 0) {
      plan.obstacle_side[index] = side;
      --hold;
    } else {
      plan.obstacle_side[index] = 0.0f;
    }
  }
}

// Backward-pass tick-space: giảm ga trước khúc cua (ràng buộc phanh phi-cục bộ).
void bake_throttle(TrackPlan& plan, const TuneConfig& cfg) {
  float next = 1.0f;
  for (std::size_t i = plan.length; i > 0; --i) {
    const std::size_t index = i - 1;
    const float grip = 1.0f - cfg.corner_slowdown * std::abs(plan.steer_ff[index]);
    const float capped = std::min(grip, next + cfg.decel_per_tick);
    plan.throttle_max[index] = std::max(0.0f, capped);
    next = plan.throttle_max[index];
  }
}

}  // namespace

float TrackPlan::steer_at(float tick) const {
  return interp(steer_ff.data(), length, tick);
}

float TrackPlan::obstacle_at(float tick) const {
  return interp(obstacle_side.data(), length, tick);
}

float TrackPlan::throttle_at(float tick) const {
  return interp(throttle_max.data(), length, tick);
}

void record(TrackPlan& plan, float tick, float steer, float obstacle_side) {
  if (tick < 0.0f) {
    return;
  }
  const auto index = static_cast<std::size_t>(std::lround(tick));
  if (index >= kMaxPlanTicks) {
    return;  // vượt dung lượng (không xảy ra với sa bàn ≤ 41m)
  }
  // Điền khoảng trống bằng giá trị trước đó.
  const float prev_steer = (plan.length > 0) ? plan.steer_ff[plan.length - 1] : 0.0f;
  const float prev_side = (plan.length > 0) ? plan.obstacle_side[plan.length - 1] : 0.0f;
  for (std::size_t i = plan.length; i < index; ++i) {
    plan.steer_ff[i] = prev_steer;
    plan.obstacle_side[i] = prev_side;
  }
  plan.steer_ff[index] = steer;
  plan.obstacle_side[index] = obstacle_side;
  plan.length = std::max(plan.length, index + 1);
}

void bake(TrackPlan& plan, const TuneConfig& cfg) {
  if (plan.length == 0) {
    return;
  }
  dilate_obstacle_side(plan, cfg);
  bake_throttle(plan, cfg);
  plan.baked = true;
}

}  // namespace lf::kernel
