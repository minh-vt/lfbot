#pragma once

#include <driver/i2c_master.h>

#include <cstdint>

#include "types.hpp"

namespace lf::esp32 {

// MPU-6500/9250 family over I2C, gyro Z only: it is the one axis the chassis
// turns about. Owned by the sensors task; reads never run inside the control tick.
class Imu {
public:
  void initialize(i2c_master_bus_handle_t bus);
  void read(std::uint64_t now_us, Sample<float>& gyro_z);
  [[nodiscard]] bool ready() const { return ready_; }
  [[nodiscard]] std::uint8_t identity() const { return identity_; }

private:
  static constexpr std::uint32_t kPeriodUs = 2000;
  i2c_master_dev_handle_t device_ = nullptr;
  std::uint8_t identity_ = 0;
  std::uint32_t sequence_ = 0;
  bool ready_ = false;
};

}  // namespace lf::esp32
