#include "line_sensor.hpp"

#include <algorithm>
#include <cmath>

#include "config.hpp"

namespace lf {

float LineCalibration::eye_position(std::size_t index) {
  // Eyes are evenly spaced, so the position is just the index relative to the
  // centre. No table to keep in step with kIrCount.
  return static_cast<float>(index) - 0.5F * static_cast<float>(kIrCount - 1);
}

LineCalibration make_calibration(const std::array<float, kIrCount>& white,
                                 const std::array<float, kIrCount>& black) {
  LineCalibration calibration;
  calibration.white = white;
  calibration.black = black;
  calibration.valid = true;
  for (std::size_t eye = 0; eye < kIrCount; ++eye) {
    const float reference = std::max(std::abs(white[eye]), 1.0F);
    const float span = std::abs(white[eye] - black[eye]);
    if (span < kLineMinContrast * reference) {
      calibration.valid = false;
    }
  }
  return calibration;
}

LineReading read_line(const LineCalibration& calibration,
                      const std::array<std::uint16_t, kIrCount>& raw) {
  LineReading reading;
  if (!calibration.valid) {
    return reading;
  }
  float weighted = 0.0F;
  float total = 0.0F;
  bool on_line = false;
  for (std::size_t eye = 0; eye < kIrCount; ++eye) {
    // Darker than white, normalised by that eye's own span. Polarity is taken
    // from the measurement, not assumed: if black reads higher than white the
    // sign flips on its own.
    const float span = calibration.white[eye] - calibration.black[eye];
    if (std::abs(span) < 1.0F) {
      continue;
    }
    const float darkness = (calibration.white[eye] - static_cast<float>(raw[eye])) / span;
    if (darkness <= kLineSplitFraction) {
      continue;
    }
    on_line = true;
    weighted += darkness * LineCalibration::eye_position(eye);
    total += darkness;
  }
  reading.valid = true;
  reading.on_line = on_line;
  reading.position = total > 0.0F ? weighted / total : 0.0F;
  return reading;
}

}  // namespace lf
