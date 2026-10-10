#pragma once

#include <algorithm>

#include "kernel/types.hpp"

namespace lf::kernel {

// Chuẩn hóa 3 tia ToF (front, trái, phải) → obstacle_danger/obstacle_side của frame.
// range_mm là tầm hữu dụng (layout::kTofRangeMm) — kernel không tự giữ đơn vị mm.
inline void normalize_tof(SensorFrame& frame, float range_mm, float d_front_mm, float d_left_mm,
                          float d_right_mm) {
  const float d_min = std::min({d_front_mm, d_left_mm, d_right_mm});
  frame.obstacle_danger = 0.0f;
  frame.obstacle_side = 0.0f;
  if (d_min >= range_mm) {
    return;
  }
  frame.obstacle_danger = std::clamp(1.0f - d_min / range_mm, 0.0f, 1.0f);
  if (d_left_mm < d_right_mm) {
    frame.obstacle_side = -1.0f;
  } else if (d_right_mm < d_left_mm) {
    frame.obstacle_side = 1.0f;
  }
}

}
