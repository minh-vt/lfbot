#pragma once

#include "drive/types.hpp"

namespace lf::esp32 {

// Khởi tạo MCPWM cho 2 cầu H.
void actuators_init();

// DriveCommand ([-1,1]) → MCPWM duty (complementary cầu H). Brake → phanh động (short motor).
void actuators_write(const lf::drive::DriveCommand& cmd);

}
