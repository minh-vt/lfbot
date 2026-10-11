#pragma once

#include "drive/types.hpp"

namespace lf::esp32 {

// Giao diện phần cứng (định nghĩa trong sensors.cpp; chỉ build trên ESP32).
void sensors_init();
void sensors_read(lf::drive::Readings& frame, float dt);

}
