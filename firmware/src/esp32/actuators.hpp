#pragma once

#include "kernel/types.hpp"

namespace lf::esp32 {

// Khởi tạo MCPWM cho 2 cầu H.
void actuators_init();

// MotorCommand ([-1,1]) → MCPWM duty (complementary cầu H). Brake → dừng.
void actuators_write(const lf::kernel::MotorCommand& cmd);

}
