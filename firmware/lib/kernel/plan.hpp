#pragma once

#include "kernel/types.hpp"

namespace lf::kernel {

// Run 1: ghi quan sát tại tick (steer là path_steer, không gồm né vật cản).
void record(TrackPlan& plan, float tick, float steer, float obstacle_side);

// Sau Run 1: giãn obstacle_side + backward-pass tick-space → throttle_max; đặt baked = true.
void bake(TrackPlan& plan, const TuneConfig& cfg);

}
