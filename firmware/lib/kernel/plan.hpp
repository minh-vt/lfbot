#pragma once

#include <cstddef>
#include <vector>

#include "kernel/types.hpp"

namespace lf::kernel {

// Run 1: ghi quan sát tại tick (steer là path_steer, không gồm né vật cản).
void record(TrackPlan& plan, float tick, float steer, float obstacle_side);

// Sau Run 1: làm mượt obstacle_side + backward-pass tick-space → throttle_max.
void bake(TrackPlan& plan, const TuneConfig& cfg);

// Nội suy tuyến tính; trả 0 nếu mảng rỗng.
[[nodiscard]] float lookup(const std::vector<float>& values, float tick);

}  // namespace lf::kernel
