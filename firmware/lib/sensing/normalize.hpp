#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "drive/types.hpp"
#include "sensing/layout.hpp"

namespace lf::sensing {

// Full-scale ADC 12-bit (2^12 − 1): vạch tối → ADC thấp.
inline constexpr float kAdcFullScale = 4095.0f;

// Chuẩn hóa thuần (test được trên native, không phụ thuộc IDF).
// Giả định phân cực IR: vạch tối → ADC thấp → "darkness" cao.
struct IrReadings {
  std::array<std::uint16_t, 8> raw{};
};

struct IrResult {
  float line_offset = 0.0f;     // [-1,1]: dương = vạch bên phải
  float line_certainty = 0.0f;  // [0,1]: độ tin cậy vạch (liên tục)
};

inline IrResult normalize_ir(const IrReadings& readings, const RobotLayout& layout) {
  float weight_sum = 0.0f;
  float weighted_pos = 0.0f;
  for (std::size_t i = 0; i < layout.ir_lateral_mm.size(); ++i) {
    const float weight = (kAdcFullScale - static_cast<float>(readings.raw[i])) / kAdcFullScale;
    weight_sum += weight;
    weighted_pos += weight * layout.ir_lateral_mm[i];
  }
  IrResult result;
  result.line_certainty = std::clamp(weight_sum, 0.0f, 1.0f);
  if (weight_sum < 1e-3f) {
    return result;  // certainty 0, offset 0
  }
  result.line_offset =
    std::clamp((weighted_pos / weight_sum) / ir_half_span_mm(layout), -1.0f, 1.0f);
  return result;
}

// Chuẩn hóa 3 tia ToF (front, trái, phải) → obstacle_danger/obstacle_side liên tục.
inline void normalize_tof(lf::drive::Readings& frame, const RobotLayout& layout, float d_front_mm,
                          float d_left_mm, float d_right_mm) {
  const float range_mm = layout.tof_range_mm;
  const float d_min = std::min({d_front_mm, d_left_mm, d_right_mm});
  frame.obstacle_danger = 0.0f;
  frame.obstacle_side = 0.0f;
  if (d_min >= range_mm) {
    return;
  }
  frame.obstacle_danger = std::clamp(1.0f - d_min / range_mm, 0.0f, 1.0f);
  // Hướng lệch liên tục: chênh lệch trái/phải chuẩn hóa theo tổng (không if/else ±1).
  // obstacle trái (d_left < d_right) → âm; phải → dương; chính diện → 0.
  const float side = (d_left_mm - d_right_mm) / std::max(d_left_mm + d_right_mm, 1e-3f);
  frame.obstacle_side = std::clamp(side, -1.0f, 1.0f);
}

}
