#include "imu.hpp"

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <array>
#include <cmath>

namespace lf::esp32 {
namespace {

constexpr std::uint8_t kRegWhoAmI = 0x75;
constexpr std::uint8_t kRegGyroZ = 0x47;
constexpr std::uint8_t kPwr1 = 0x6B;
constexpr std::uint8_t kConfig = 0x1A;
constexpr std::uint8_t kSampleRate = 0x19;
constexpr std::uint8_t kGyroConfig = 0x1B;
constexpr std::uint8_t kRegPwr2 = 0x6C;
constexpr float kRadiansPerDegree = 0.01745329252F;
// +-1000 dps keeps a fast spin on scale; 32.8 LSB per dps at that range.
constexpr std::uint8_t kGyroRange1000Dps = 0x10;
constexpr float kGyroLsbPerDps = 32.8F;
// 92 Hz filter, 1 kHz internal rate divided to 500 Hz.
constexpr std::uint8_t kFilter92Hz = 0x02;
constexpr std::uint8_t kAccelOff = 0x38;
constexpr int kClippedCount = 32760;

// MPU-6500 (0x70), MPU-9250 (0x71) and MPU-9255 (0x73) share these registers.
// GY-9250 boards are often shipped with a 6500 fitted, so the family is
// accepted and the magnetometer is never read.
bool supported(std::uint8_t identity) {
  return identity == 0x70 || identity == 0x71 || identity == 0x73;
}

bool read_regs(i2c_master_dev_handle_t device, std::uint8_t reg, std::uint8_t* bytes,
               std::size_t count) {
  return i2c_master_transmit_receive(device, &reg, 1, bytes, count, 10) == ESP_OK;
}

bool write_reg(i2c_master_dev_handle_t device, std::uint8_t reg, std::uint8_t value) {
  const std::array<std::uint8_t, 2> bytes = {reg, value};
  return i2c_master_transmit(device, bytes.data(), bytes.size(), 10) == ESP_OK;
}

}  // namespace

void Imu::initialize(i2c_master_bus_handle_t bus) {
  i2c_device_config_t config{};
  config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  config.device_address = 0x68;
  config.scl_speed_hz = 400000;
  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &config, &device_));
  // The sensor may still be booting when the ESP32 is ready, so the identity is
  // retried instead of being decided once and wrong for the whole session.
  for (int attempt = 0; attempt < 5; ++attempt) {
    if (read_regs(device_, kRegWhoAmI, &identity_, 1) && supported(identity_))
      break;
    identity_ = 0;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  if (identity_ == 0)
    return;
  ready_ = write_reg(device_, kPwr1, 0x80);  // reset
  vTaskDelay(pdMS_TO_TICKS(100));
  ready_ = ready_ && write_reg(device_, kPwr1, 0x01) && write_reg(device_, kRegPwr2, kAccelOff) &&
           write_reg(device_, kConfig, kFilter92Hz) && write_reg(device_, kSampleRate, 0x01) &&
           write_reg(device_, kGyroConfig, kGyroRange1000Dps);
}

void Imu::read(std::uint64_t now_us, Sample<float>& gyro_z) {
  if (!ready_) {
    gyro_z.status = SampleStatus::kInvalid;
    return;
  }
  std::array<std::uint8_t, 2> raw{};
  if (!read_regs(device_, kRegGyroZ, raw.data(), raw.size())) {
    gyro_z.status = SampleStatus::kInvalid;
    return;
  }
  const unsigned word = (static_cast<unsigned>(raw[0]) << 8U) | raw[1];
  const int value = word >= 32768U ? static_cast<int>(word) - 65536 : static_cast<int>(word);
  ++sequence_;
  gyro_z.value = static_cast<float>(value) * kRadiansPerDegree / kGyroLsbPerDps;
  gyro_z.sequence = sequence_;
  gyro_z.status = std::abs(value) >= kClippedCount ? SampleStatus::kInvalid : SampleStatus::kValid;
  gyro_z.earliest_us = now_us > kPeriodUs ? now_us - kPeriodUs : 0;
  gyro_z.latest_us = now_us;
  gyro_z.period_us = kPeriodUs;
}

}  // namespace lf::esp32
