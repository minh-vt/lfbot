#pragma once

#include <array>
#include <cstdint>

#include "types.hpp"

namespace lf {

// What each eye reads on white and on black. Invalid when some eye cannot tell
// them apart, because that eye would feed noise into the steering.
struct LineCalibration {
  std::array<float, kIrCount> white{};
  std::array<float, kIrCount> black{};
  bool valid = false;

  // 0 = the line is centred under the bar, -3.5 = far left eye, +3.5 = far right.
  [[nodiscard]] static float eye_position(std::size_t index);
};

struct LineReading {
  bool valid = false;    // calibration usable and nothing saturated
  bool on_line = false;  // at least one eye clearly sees the line
  float position = 0.0F;
};

LineCalibration make_calibration(const std::array<float, kIrCount>& white,
                                 const std::array<float, kIrCount>& black);

LineReading read_line(const LineCalibration& calibration,
                      const std::array<std::uint16_t, kIrCount>& raw);

}  // namespace lf
