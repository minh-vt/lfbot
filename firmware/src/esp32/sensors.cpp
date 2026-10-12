#include "sensors.hpp"

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <driver/pulse_cnt.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "drive/types.hpp"
#include "sensing/normalize.hpp"

namespace lf::esp32 {
namespace {

// Pin map (docs/HARDWARE_GUIDE.md).
constexpr std::array<adc_channel_t, 8> kIrChannel = {ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2,
                                                     ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_5,
                                                     ADC_CHANNEL_6, ADC_CHANNEL_7};

constexpr int kEncLeftA = 15, kEncLeftB = 16;
constexpr int kEncRightA = 17, kEncRightB = 18;

constexpr std::uint32_t kFinishPin = 22;  // GIẢ ĐỊNH (chưa có trong HARDWARE_GUIDE)

constexpr int kI2cSda = 11, kI2cScl = 12;
constexpr int kTofXshutFront = 13, kTofXshutRight = 14, kTofXshutLeft = 21;
constexpr std::uint8_t kTofAddrFront = 0x30, kTofAddrRight = 0x31, kTofAddrLeft = 0x32;

struct Hardware {
  adc_oneshot_unit_handle_t adc = nullptr;
  pcnt_unit_handle_t enc_left = nullptr;
  pcnt_unit_handle_t enc_right = nullptr;
  i2c_master_bus_handle_t i2c_bus = nullptr;
  i2c_master_dev_handle_t tof_front = nullptr;
  i2c_master_dev_handle_t tof_right = nullptr;
  i2c_master_dev_handle_t tof_left = nullptr;
};
Hardware g_hw;

// ToF cập nhật nền; vòng 500Hz chỉ đọc cache (atomic), không bao giờ block.
std::atomic<float> g_tof_front_mm{sensing::kTofRangeMm};
std::atomic<float> g_tof_left_mm{sensing::kTofRangeMm};
std::atomic<float> g_tof_right_mm{sensing::kTofRangeMm};

// PCNT encoder (quadrature).
void init_encoder(int gpio_a, int gpio_b, pcnt_unit_handle_t* unit) {
  pcnt_unit_config_t unit_cfg = {};
  unit_cfg.high_limit = 30000;
  unit_cfg.low_limit = -30000;
  pcnt_new_unit(&unit_cfg, unit);

  pcnt_glitch_filter_config_t filter = {};
  filter.max_glitch_ns = 1000;
  pcnt_unit_set_glitch_filter(*unit, &filter);

  pcnt_chan_config_t chan_cfg = {};
  chan_cfg.edge_gpio_num = gpio_a;
  chan_cfg.level_gpio_num = gpio_b;
  pcnt_channel_handle_t channel = nullptr;
  pcnt_new_channel(*unit, &chan_cfg, &channel);
  pcnt_channel_set_edge_action(channel, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                               PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  pcnt_channel_set_level_action(channel, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

  pcnt_unit_enable(*unit);
  pcnt_unit_clear_count(*unit);
  pcnt_unit_start(*unit);
}

std::int32_t read_encoder(pcnt_unit_handle_t unit) {
  int value = 0;
  pcnt_unit_get_count(unit, &value);
  return value;
}

// VL53L0X (I2C master IDF 6.1).
// CẢNH BÁO: chuỗi init/calibration đầy đủ cần đối chiếu datasheet trên phần cứng.
i2c_master_dev_handle_t add_tof(std::uint8_t address) {
  i2c_device_config_t dev_cfg = {};
  dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_cfg.device_address = address;
  dev_cfg.scl_speed_hz = 400000;
  i2c_master_dev_handle_t device = nullptr;
  i2c_master_bus_add_device(g_hw.i2c_bus, &dev_cfg, &device);
  return device;
}

std::uint8_t tof_read_reg(i2c_master_dev_handle_t device, std::uint8_t reg) {
  std::uint8_t value = 0;
  i2c_master_transmit_receive(device, &reg, 1, &value, 1, 20);
  return value;
}

void tof_write_reg(i2c_master_dev_handle_t device, std::uint8_t reg, std::uint8_t value) {
  const std::array<std::uint8_t, 2> payload = {reg, value};
  i2c_master_transmit(device, payload.data(), payload.size(), 20);
}

float tof_read_mm(i2c_master_dev_handle_t device) {
  if (device == nullptr) {
    return sensing::kTofRangeMm;
  }
  tof_write_reg(device, 0x00, 0x01);  // SYSRANGE_START: single-shot
  vTaskDelay(pdMS_TO_TICKS(30));
  const std::uint8_t msb = tof_read_reg(device, 0x1E);
  const std::uint8_t lsb = tof_read_reg(device, 0x1F);
  tof_write_reg(device, 0x0B, 0x01);  // SYSTEM_INTERRUPT_CLEAR
  const auto mm =
    static_cast<float>((static_cast<std::uint16_t>(msb) << 8) | static_cast<std::uint16_t>(lsb));
  return (mm > 0.0f && mm < 2000.0f) ? mm : sensing::kTofRangeMm;
}

// Quét 3 ToF nền (~30ms/con) → ghi cache. Vòng điều khiển chỉ đọc cache.
void tof_task(void*) {
  for (;;) {
    g_tof_front_mm.store(tof_read_mm(g_hw.tof_front));
    g_tof_left_mm.store(tof_read_mm(g_hw.tof_left));
    g_tof_right_mm.store(tof_read_mm(g_hw.tof_right));
  }
}

}

void sensors_init() {
  adc_oneshot_unit_init_cfg_t adc_cfg = {};
  adc_cfg.unit_id = ADC_UNIT_1;
  adc_oneshot_new_unit(&adc_cfg, &g_hw.adc);
  adc_oneshot_chan_cfg_t chan_cfg = {};
  chan_cfg.atten = ADC_ATTEN_DB_12;
  chan_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
  for (const auto channel : kIrChannel) {
    adc_oneshot_config_channel(g_hw.adc, channel, &chan_cfg);
  }

  init_encoder(kEncLeftA, kEncLeftB, &g_hw.enc_left);
  init_encoder(kEncRightA, kEncRightB, &g_hw.enc_right);

  gpio_config_t finish = {};
  finish.pin_bit_mask = 1ULL << kFinishPin;
  finish.mode = GPIO_MODE_INPUT;
  finish.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_config(&finish);

  i2c_master_bus_config_t bus_cfg = {};
  bus_cfg.i2c_port = I2C_NUM_0;
  bus_cfg.sda_io_num = static_cast<gpio_num_t>(kI2cSda);
  bus_cfg.scl_io_num = static_cast<gpio_num_t>(kI2cScl);
  bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_cfg.glitch_ignore_cnt = 7;
  bus_cfg.flags.enable_internal_pullup = 1;
  i2c_new_master_bus(&bus_cfg, &g_hw.i2c_bus);
  g_hw.tof_front = add_tof(kTofAddrFront);
  g_hw.tof_right = add_tof(kTofAddrRight);
  g_hw.tof_left = add_tof(kTofAddrLeft);

  for (const int pin : {kTofXshutFront, kTofXshutRight, kTofXshutLeft}) {
    gpio_set_direction(static_cast<gpio_num_t>(pin), GPIO_MODE_OUTPUT);
    gpio_set_level(static_cast<gpio_num_t>(pin), 0);
  }
  // TODO(hw): bật XSHUT tuần tự + gán địa chỉ + init VL53L0X (datasheet).

  xTaskCreatePinnedToCore(tof_task, "tof", 2048, nullptr, 1, nullptr, 0);
}

void sensors_read(lf::drive::Readings& frame, float dt) {
  sensing::IrReadings ir;
  for (std::size_t i = 0; i < sensing::kIrOffsetsMm.size(); ++i) {
    int raw = 0;
    adc_oneshot_read(g_hw.adc, kIrChannel[i], &raw);
    ir.raw[i] = static_cast<std::uint16_t>(raw);
  }
  const sensing::IrResult ir_result = sensing::normalize_ir(ir);
  frame.line_offset = ir_result.line_offset;
  frame.line_certainty = ir_result.line_certainty;

  sensing::normalize_tof(frame, sensing::kTofRangeMm, g_tof_front_mm.load(), g_tof_left_mm.load(),
                         g_tof_right_mm.load());

  frame.enc_l = read_encoder(g_hw.enc_left);
  frame.enc_r = read_encoder(g_hw.enc_right);
  frame.finish_detected = gpio_get_level(static_cast<gpio_num_t>(kFinishPin)) == 0;
  frame.dt = dt;
}

}
