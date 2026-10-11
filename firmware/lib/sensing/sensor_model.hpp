#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "drive/types.hpp"

namespace lf::sensing {

// Cụm 8 cảm biến IR dò vạch: vị trí ngang (mm) so với tâm cụm, dương = phải.
inline constexpr std::array<float, 8> kIrOffsetsMm = {
  -42.0f, -30.0f, -18.0f, -6.0f, 6.0f, 18.0f, 30.0f, 42.0f,
};
inline constexpr float kIrHalfSpanMm = kIrOffsetsMm.back();

// Tầm ToF hữu dụng (mm): ngưỡng danger + tầm raycast sim + fallback esp32.
inline constexpr float kTofRangeMm = 300.0f;

// Chuẩn hóa thuần (test được trên native, không phụ thuộc IDF).
// Giả định phân cực IR: vạch tối → ADC thấp → "darkness" cao.
struct IrReadings {
  std::array<std::uint16_t, 8> raw{};
};

struct IrResult {
  float line_offset = 0.0f;     // [-1,1]: dương = vạch bên phải
  float line_certainty = 0.0f;  // [0,1]: độ tin cậy vạch (liên tục)
};

inline IrResult normalize_ir(const IrReadings& readings) {
  constexpr float kAdcMax = 4095.0f;
  float weight_sum = 0.0f;
  float weighted_pos = 0.0f;
  for (std::size_t i = 0; i < kIrOffsetsMm.size(); ++i) {
    const float weight = (kAdcMax - static_cast<float>(readings.raw[i])) / kAdcMax;
    weight_sum += weight;
    weighted_pos += weight * kIrOffsetsMm[i];
  }
  IrResult result;
  result.line_certainty = std::clamp(weight_sum, 0.0f, 1.0f);
  if (weight_sum < 1e-3f) {
    return result;  // certainty 0, offset 0
  }
  result.line_offset = std::clamp((weighted_pos / weight_sum) / kIrHalfSpanMm, -1.0f, 1.0f);
  return result;
}

// Chuẩn hóa 3 tia ToF (front, trái, phải) → obstacle_danger/obstacle_side của frame.
inline void normalize_tof(lf::drive::Readings& frame, float range_mm, float d_front_mm,
                          float d_left_mm, float d_right_mm) {
  const float d_min = std::min({d_front_mm, d_left_mm, d_right_mm});
  frame.obstacle_danger = 0.0f;
  frame.obstacle_side = 0.0f;
  if (d_min >= range_mm) {
    return;
  }
  frame.obstacle_danger = std::clamp(1.0f - d_min / range_mm, 0.0f, 1.0f);
  if (d_left_mm < d_right_mm) {
    frame.obstacle_side = -1.0f;
  } else if (d_right_mm < d_left_mm) {
    frame.obstacle_side = 1.0f;
  }
}

}
