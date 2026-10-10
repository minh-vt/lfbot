#pragma once

#include <array>

namespace lf::layout {

// Cụm 8 cảm biến IR dò vạch: vị trí ngang (mm) so với tâm cụm, dương = phải.
// Chân lý phần cứng dùng chung cho sim (mô hình hóa) và esp32 (chuẩn hóa ADC).
inline constexpr std::array<float, 8> kIrOffsetsMm = {
  -42.0f, -30.0f, -18.0f, -6.0f, 6.0f, 18.0f, 30.0f, 42.0f,
};
inline constexpr float kIrHalfSpanMm = 42.0f;

}  // namespace lf::layout
