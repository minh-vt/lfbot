#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "kernel/types.hpp"
#include "layout/sensor_layout.hpp"

namespace lf::esp32 {

// Chuẩn hóa thuần (test được trên native, không phụ thuộc Arduino/IDF).
// Giả định phân cực IR: vạch tối → ADC thấp → "darkness" cao (calibration adapter).

struct IrReadings {
  std::array<std::uint16_t, 8> raw{};
};

struct IrResult {
  float line_error = 0.0f;  // [-1,1]: dương = vạch bên phải
  bool line_detected = false;
};

inline IrResult normalize_ir(const IrReadings& readings) {
  constexpr float kAdcMax = 4095.0f;
  float weight_sum = 0.0f;
  float weighted_pos = 0.0f;
  for (std::size_t i = 0; i < layout::kIrOffsetsMm.size(); ++i) {
    const float weight = (kAdcMax - static_cast<float>(readings.raw[i])) / kAdcMax;
    weight_sum += weight;
    weighted_pos += weight * layout::kIrOffsetsMm[i];
  }
  IrResult result;
  if (weight_sum < 1e-3f) {
    result.line_detected = false;
    result.line_error = 0.0f;
  } else {
    result.line_detected = true;
    result.line_error =
      std::clamp((weighted_pos / weight_sum) / layout::kIrHalfSpanMm, -1.0f, 1.0f);
  }
  return result;
}

struct TofResult {
  float danger = 0.0f;  // [0,1]
  float side = 0.0f;    // [-1,1]: âm = vật cản trái
};

inline TofResult normalize_tof(float d_front_mm, float d_left_mm, float d_right_mm) {
  constexpr float kRangeMm = 300.0f;
  const float d_min = std::min({d_front_mm, d_left_mm, d_right_mm});
  TofResult result;
  result.danger = (d_min < kRangeMm) ? std::clamp(1.0f - d_min / kRangeMm, 0.0f, 1.0f) : 0.0f;
  if (result.danger > 0.0f) {
    if (d_left_mm < d_right_mm) {
      result.side = -1.0f;
    } else if (d_right_mm < d_left_mm) {
      result.side = 1.0f;
    } else {
      result.side = 0.0f;
    }
  } else {
    result.side = 0.0f;
  }
  return result;
}

// Giao diện phần cứng (định nghĩa trong sensors.cpp; chỉ build trên ESP32).
void sensors_init();
void sensors_read(lf::kernel::SensorFrame& frame, float dt);

}  // namespace lf::esp32
