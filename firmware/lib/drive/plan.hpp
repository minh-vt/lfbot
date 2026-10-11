#pragma once

#include "drive/types.hpp"

namespace lf::drive {

// Run 1: ghi quan sát tại tick (steer là path_steer, không gồm né vật cản).
void record(RacePlan& plan, float tick, float steer, float obstacle_side);

// Sau Run 1: giãn obstacle_side + backward-pass tick-space → speed_limit; đặt ready = true.
void plan_speed(RacePlan& plan, const Tuning& cfg);

}
