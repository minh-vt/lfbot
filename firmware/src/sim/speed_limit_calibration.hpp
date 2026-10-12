#pragma once

#include <algorithm>
#include <cmath>

#include "drive/types.hpp"

namespace lf::sim {

// Sinh bảng slip-boundary từ oracle tracking-limit (ticket 34).
// Trục bảng = |steer| ∈ [0,1] (cùng đơn vị `line_curve`). κ ước lượng = steer/geometry_l
// (u_ss = L·κ khi bám đều), nên:
//   v(steer) = ωn·√(s_tick·budget/κ) = ωn·√(s_tick·budget·geometry_l/steer)   [tick/s]
// chuẩn hoá về v_max_tick.
inline drive::SpeedLimitTable calibrate_speed_limit(const drive::RobotTuning& robot,
                                                    float v_max_tick) {
  constexpr float kBudget = 0.35f;  // e_ss cho phép (biên tới mất vạch tại |e|=1)
  drive::SpeedLimitTable table;
  constexpr int kPoints = 10;
  for (int i = 0; i <= kPoints; ++i) {
    const float steer = static_cast<float>(i) / static_cast<float>(kPoints);
    float limit = 1.0f;
    if (steer > 1e-3f) {
      const float v =
        robot.bandwidth * std::sqrt(robot.geometry_s * kBudget * robot.geometry_l / steer);
      limit = std::clamp(v / v_max_tick, 0.0f, 1.0f);
    }
    table.add(steer, limit);
  }
  return table;
}

}
