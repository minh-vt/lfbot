#pragma once

#include "drive/types.hpp"

namespace lf::drive {

// Run 1: ghi quỹ đạo tại tick (lateral = vị trí ngang đã đạt; speed = tốc độ đã đạt).
void record(RacePlan& plan, float tick, float lateral, float speed, float obstacle_side);

// Sau Run 1: giãn obstacle_side + backward-pass tick-space → speed_limit; đặt ready = true.
void plan_speed(RacePlan& plan, const Tuning& cfg);

}
