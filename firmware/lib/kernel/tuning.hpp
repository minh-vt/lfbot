#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "kernel/types.hpp"

namespace lf::kernel {

// Tầng 1 — suy ra từ dt / hình học / quan sát Run 1 (không chỉnh tay).

// alpha lọc đạo hàm (low-pass bậc 1) tại tần số cắt cutoff_hz, bước dt.
inline float derive_d_filter_alpha(float dt, float cutoff_hz) {
  constexpr float kTwoPi = 6.28318530717958647692f;
  const float rc = 1.0f / (kTwoPi * cutoff_hz);
  return dt / (dt + rc);
}

// Số tick giãn obstacle_side để phủ quãng đường span_mm, với 1 tick = mm_per_tick.
inline std::int32_t derive_obstacle_window(float span_mm, float mm_per_tick) {
  const float ticks = span_mm / mm_per_tick;
  return std::max<std::int32_t>(1, static_cast<std::int32_t>(std::lround(ticks)));
}

// decel_per_tick = mức giảm throttle/tick lớn nhất thực thi được (đo ở Run 1).
inline float derive_decel_per_tick(float max_drop_per_tick) {
  return std::max(0.0f, max_drop_per_tick);
}

// Tầng 2 — pole-placement cho vòng PD bậc 2 (ζ, ωn) với gain plant K (steer→heading-rate):
// kp = ωn² / K, kd = 2ζωn / K.
inline void derive_pd_gains(float zeta, float wn, float plant_gain, float* kp, float* kd) {
  *kp = wn * wn / plant_gain;
  *kd = 2.0f * zeta * wn / plant_gain;
}

// Tầng 3 — trim sprint_scale theo số lần mất vạch ở Run 1 (monotonic, sàn 1.0).
inline float trim_sprint_scale(float current, int line_losses, float step = 0.05f) {
  const float delta = step * ((line_losses == 0) ? 1.0f : -static_cast<float>(line_losses));
  return std::max(1.0f, current + delta);
}

}
