#pragma once

#include <array>

namespace lf::layout {

// Cụm 8 cảm biến IR dò vạch: vị trí ngang (mm) so với tâm cụm, dương = phải.
inline constexpr std::array<float, 8> kIrOffsetsMm = {
  -42.0f, -30.0f, -18.0f, -6.0f, 6.0f, 18.0f, 30.0f, 42.0f,
};
inline constexpr float kIrHalfSpanMm = kIrOffsetsMm.back();

// Tầm ToF hữu dụng (mm): ngưỡng danger + tầm raycast sim + fallback esp32.
inline constexpr float kTofRangeMm = 300.0f;

}
