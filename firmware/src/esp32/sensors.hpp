#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "kernel/normalize.hpp"
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

// Giao diện phần cứng (định nghĩa trong sensors.cpp; chỉ build trên ESP32).
void sensors_init();
void sensors_read(lf::kernel::SensorFrame& frame, float dt);

}
