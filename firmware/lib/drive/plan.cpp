#include "drive/plan.hpp"

#include <algorithm>
#include <cmath>

namespace lf::drive {
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
void dilate_obstacle_side(RacePlan& plan, const Tuning& cfg) {
  const int span = 2 * std::max(1, static_cast<int>(cfg.track.obstacle_window));
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

// Backward-pass tick-space: speed_limit(t) = min(speed_limit(line_curve(t)),
// speed_limit(t+1)+brake).
void plan_speed_pass(RacePlan& plan, const Tuning& cfg) {
  float next = 1.0f;
  for (std::size_t i = plan.length; i > 0; --i) {
    const std::size_t index = i - 1;
    const float grip = cfg.robot.speed_limit(plan.line_curve[index]);
    const float capped = std::min(grip, next + cfg.track.brake_limit);
    plan.speed_limit[index] = std::max(0.0f, capped);
    next = plan.speed_limit[index];
  }
}

// Racing-line offset (ticket 25): cắt trong cua = nửa span (geometry, không gain tay).
void plan_offset_pass(RacePlan& plan) {
  constexpr float kRacingLineOffset = 0.0f;  // placeholder ticket 25 tạm tắt
  for (std::size_t i = 0; i < plan.length; ++i) {
    plan.offset_setpoint[i] = -kRacingLineOffset * plan.line_curve[i];
  }
}

}

// Truy cập plan trả giá trị TRUNG TÍNH khi chưa ready (đang ghi hoặc chưa plan):
// feedforward/setpoint/obstacle → 0; speed_limit → 1 (không chặn). Nhờ vậy step()
// không cần rẽ nhánh `plan.ready` cho setpoint/feedforward.
float RacePlan::line_curve_at(float tick) const {
  return ready ? interp(line_curve.data(), length, tick) : 0.0f;
}

float RacePlan::obstacle_at(float tick) const {
  return ready ? interp(obstacle_side.data(), length, tick) : 0.0f;
}

float RacePlan::speed_limit_at(float tick) const {
  return ready ? interp(speed_limit.data(), length, tick) : 1.0f;
}

float RacePlan::offset_at(float tick) const {
  return ready ? interp(offset_setpoint.data(), length, tick) : 0.0f;
}

void record(RacePlan& plan, float tick, float steer, float obstacle_side) {
  if (tick < 0.0f) {
    return;
  }
  const auto index = static_cast<std::size_t>(std::lround(tick));
  if (index >= kMaxPlanTicks) {
    return;  // vượt dung lượng (không xảy ra với sa bàn ≤ 41m)
  }
  // Điền khoảng trống bằng giá trị trước đó.
  const float prev_side = (plan.length > 0) ? plan.obstacle_side[plan.length - 1] : 0.0f;
  const float prev_curve = (plan.length > 0) ? plan.line_curve[plan.length - 1] : 0.0f;
  for (std::size_t i = plan.length; i < index; ++i) {
    plan.obstacle_side[i] = prev_side;
    plan.line_curve[i] = prev_curve;
  }
  // line_curve = geometry thuần (low-pass tín hiệu lái, bỏ nhiễu PD).
  constexpr float kCurveSmoothing = 0.3f;
  plan.obstacle_side[index] = obstacle_side;
  plan.line_curve[index] = kCurveSmoothing * steer + (1.0f - kCurveSmoothing) * prev_curve;
  plan.length = std::max(plan.length, index + 1);
}

void plan_speed(RacePlan& plan, const Tuning& cfg) {
  if (plan.length == 0) {
    return;
  }
  dilate_obstacle_side(plan, cfg);
  plan_speed_pass(plan, cfg);
  plan_offset_pass(plan);
  plan.ready = true;
}

}
