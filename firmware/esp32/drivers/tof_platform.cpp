#include <driver/i2c_master.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <array>
#include <cstdint>
#include <cstring>

extern "C" {
#include "vl53l1x_api.h"
}

extern "C" int8_t VL53L1_WriteMulti(VL53L1_DEV dev, uint16_t index, uint8_t* data, uint32_t count) {
  std::array<std::uint8_t, 258> bytes{};
  if (dev->failed != 0 || count > bytes.size() - 2) {
    dev->failed = 1;
    return -1;
  }
  bytes[0] = static_cast<std::uint8_t>(index >> 8);
  bytes[1] = static_cast<std::uint8_t>(index);
  std::memcpy(bytes.data() + 2, data, count);
  if (i2c_master_transmit(static_cast<i2c_master_dev_handle_t>(dev->device), bytes.data(),
                          count + 2, 10) != ESP_OK)
    dev->failed = 1;
  return dev->failed == 0 ? 0 : -1;
}

extern "C" int8_t VL53L1_ReadMulti(VL53L1_DEV dev, uint16_t index, uint8_t* data, uint32_t count) {
  std::memset(data, 0, count);
  const std::array<std::uint8_t, 2> address = {static_cast<std::uint8_t>(index >> 8),
                                               static_cast<std::uint8_t>(index)};
  if (dev->failed == 0 &&
      i2c_master_transmit_receive(static_cast<i2c_master_dev_handle_t>(dev->device), address.data(),
                                  address.size(), data, count, 10) != ESP_OK)
    dev->failed = 1;
  return dev->failed == 0 ? 0 : -1;
}

extern "C" int8_t VL53L1_WrByte(VL53L1_DEV dev, uint16_t index, uint8_t data) {
  return VL53L1_WriteMulti(dev, index, &data, 1);
}
extern "C" int8_t VL53L1_WrWord(VL53L1_DEV dev, uint16_t index, uint16_t data) {
  std::array<std::uint8_t, 2> bytes = {static_cast<std::uint8_t>(data >> 8),
                                       static_cast<std::uint8_t>(data)};
  return VL53L1_WriteMulti(dev, index, bytes.data(), bytes.size());
}
extern "C" int8_t VL53L1_WrDWord(VL53L1_DEV dev, uint16_t index, uint32_t data) {
  std::array<std::uint8_t, 4> bytes = {
    static_cast<std::uint8_t>(data >> 24), static_cast<std::uint8_t>(data >> 16),
    static_cast<std::uint8_t>(data >> 8), static_cast<std::uint8_t>(data)};
  return VL53L1_WriteMulti(dev, index, bytes.data(), bytes.size());
}
extern "C" int8_t VL53L1_RdByte(VL53L1_DEV dev, uint16_t index, uint8_t* data) {
  return VL53L1_ReadMulti(dev, index, data, 1);
}
extern "C" int8_t VL53L1_RdWord(VL53L1_DEV dev, uint16_t index, uint16_t* data) {
  std::array<std::uint8_t, 2> bytes{};
  const auto result = VL53L1_ReadMulti(dev, index, bytes.data(), bytes.size());
  *data = static_cast<std::uint16_t>((bytes[0] << 8) | bytes[1]);
  return result;
}
extern "C" int8_t VL53L1_RdDWord(VL53L1_DEV dev, uint16_t index, uint32_t* data) {
  std::array<std::uint8_t, 4> bytes{};
  const auto result = VL53L1_ReadMulti(dev, index, bytes.data(), bytes.size());
  *data = (static_cast<std::uint32_t>(bytes[0]) << 24) |
          (static_cast<std::uint32_t>(bytes[1]) << 16) |
          (static_cast<std::uint32_t>(bytes[2]) << 8) | bytes[3];
  return result;
}

namespace {
int8_t wait_sensor(VL53L1_DEV dev, bool boot) {
  const auto deadline = esp_timer_get_time() + 500000;
  while (esp_timer_get_time() < deadline && dev->failed == 0) {
    std::uint8_t ready = 0;
    const auto status =
      boot ? VL53L1X_BootState(dev, &ready) : VL53L1X_CheckForDataReady(dev, &ready);
    if (status != 0)
      return status;
    if (ready != 0)
      return 0;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  dev->failed = 1;
  return -1;
}
}  // namespace

extern "C" int8_t VL53L1_WaitReady(VL53L1_DEV dev) {
  return wait_sensor(dev, false);
}
extern "C" int8_t VL53L1_WaitBoot(VL53L1_DEV dev) {
  return wait_sensor(dev, true);
}
