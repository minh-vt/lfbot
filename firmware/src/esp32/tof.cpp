#include "tof.hpp"

#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern "C" {
#include "vl53l1x_api.h"
}

#include "pins.hpp"

namespace lf::esp32 {

void Tof::initialize(i2c_master_bus_handle_t bus) {
  gpio_config_t shutdown{};
  shutdown.pin_bit_mask = 1ULL << pins::kTofShutdown;
  shutdown.mode = GPIO_MODE_OUTPUT_OD;
  ESP_ERROR_CHECK(gpio_config(&shutdown));
  ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_41, 0));
  vTaskDelay(pdMS_TO_TICKS(2));
  ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_41, 1));
  vTaskDelay(pdMS_TO_TICKS(10));

  i2c_device_config_t config{};
  config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  config.device_address = 0x29;
  config.scl_speed_hz = 400000;
  i2c_master_dev_handle_t handle = nullptr;
  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &config, &handle));
  device_.device = handle;
  device_.i2c_trans.slave_addr = 0x52;  // ULD convention; the IDF handle is 7-bit.
  ready_ = VL53L1_WaitBoot(&device_) == 0 && VL53L1X_GetSensorId(&device_, &identity_) == 0 &&
           identity_ == 0xEACC && VL53L1X_SensorInit(&device_) == 0 &&
           VL53L1X_SetDistanceMode(&device_, 1) == 0 &&
           VL53L1X_SetTimingBudgetInMs(&device_, 20) == 0 &&
           VL53L1X_SetInterMeasurementInMs(&device_, 25) == 0 &&
           VL53L1X_StartRanging(&device_) == 0 && device_.failed == 0;
  earliest_us_ = static_cast<std::uint64_t>(esp_timer_get_time());
}

void Tof::read(std::uint64_t now_us, Sample<float>& sample, std::uint8_t& range_status) {
  if (!ready_) {
    sample.status = SampleStatus::kInvalid;
    return;
  }
  std::uint8_t data_ready = 0;
  device_.failed = 0;
  if (VL53L1X_CheckForDataReady(&device_, &data_ready) != 0 || device_.failed != 0) {
    sample.status = SampleStatus::kInvalid;
    return;
  }
  if (data_ready == 0)
    return;  // keep the previous sample; freshness handles age
  VL53L1X_Result_t result{};
  const auto read_status = VL53L1X_GetResult(&device_, &result);
  const auto clear_status = VL53L1X_ClearInterrupt(&device_);
  range_status = result.Status;
  ++sequence_;
  sample.value = static_cast<float>(result.Distance);
  sample.sequence = sequence_;
  sample.earliest_us = earliest_us_;
  sample.latest_us = now_us;
  sample.period_us = kPeriodUs;
  sample.status = read_status == 0 && clear_status == 0 && device_.failed == 0 &&
                      result.Status == 0 && result.Distance != 0
                    ? SampleStatus::kValid
                    : SampleStatus::kInvalid;
  earliest_us_ = now_us > kPeriodUs ? now_us - kPeriodUs : 0;
}

}  // namespace lf::esp32
