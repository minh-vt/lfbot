#pragma once

#include <driver/pulse_cnt.h>

#include <array>
#include <cstdint>

#include "types.hpp"

namespace lf::esp32 {

class Encoders {
public:
  void initialize();
  void read(std::uint64_t now_us, Sample<std::array<std::int32_t, kWheelCount>>& sample);

private:
  static constexpr std::uint32_t kPeriodUs = 2000;
  std::array<pcnt_unit_handle_t, kWheelCount> units_{};
};

}  // namespace lf::esp32
