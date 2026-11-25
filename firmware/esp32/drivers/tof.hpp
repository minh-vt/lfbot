#pragma once

#include <driver/i2c_master.h>

#include <cstdint>

extern "C" {
#include "vl53l1_platform.h"
}

#include "types.hpp"

namespace lf::esp32 {

class Tof {
public:
  void initialize(i2c_master_bus_handle_t bus);
  void read(std::uint64_t now_us, Sample<float>& sample, std::uint8_t& range_status);
  [[nodiscard]] bool ready() const { return ready_; }
  [[nodiscard]] std::uint16_t identity() const { return identity_; }

private:
  static constexpr std::uint32_t kPeriodUs = 25000;  // 20 ms budget + margin
  VL53L1_Device device_{};
  std::uint16_t identity_ = 0;
  std::uint32_t sequence_ = 0;
  std::uint64_t earliest_us_ = 0;
  bool ready_ = false;
};

}  // namespace lf::esp32
