#include "adc_inputs.hpp"

#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <soc/soc_caps.h>

#include <algorithm>
#include <cstring>

#include "pins.hpp"

namespace lf::esp32 {
namespace {

constexpr std::uint32_t kSampleFreqHz = 24000;
constexpr std::uint32_t kFrameBytes = 128;
// Switching the LEDs every tick needs to know which sweeps came after the switch; a
// frame of one sweep (9 conversions of 4 bytes) leaves at most one stale sweep queued.
constexpr std::uint32_t kSwitchedFrameBytes = (kIrCount + 1) * SOC_ADC_DIGI_DATA_BYTES_PER_CONV;
// Sweeps skipped after a switch: the one queued from before it, and one for the eyes
// to settle. UNVERIFIED: the bar's LED and phototransistor timing is not measured.
constexpr std::uint32_t kSettlingSweeps = 2;
constexpr std::uint32_t kAdcFull = 4095;
constexpr std::uint32_t kPeriodUs = 1000000U / (kSampleFreqHz / (kIrCount + 1));
// Battery divider on the base PCB: 47k over 10k, both 1%, so (47 + 10) / 10.
// A measured ratio belongs in the robot profile; this is the nominal one.
constexpr float kDefaultDivider = 5.7F;
// No complete sweep for this long means the DMA has stopped (it ran dry while the
// CPU was busy or the cache was off): start it again. Normal gap is 0.4 ms.
constexpr std::uint64_t kStallRestartUs = 5000;
constexpr std::uint64_t kRestartHoldoffUs = 20000;

}  // namespace

void AdcInputs::initialize() {
  differential_ = pins::kIrCtrlWiring != pins::IrCtrl::kNotUsed;
  if (differential_) {
    gpio_config_t pin{};
    pin.pin_bit_mask = 1ULL << pins::kIrCtrl;
    pin.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&pin));
    switch_leds();
  }
  adc_continuous_handle_cfg_t handle_config{};
  handle_config.max_store_buf_size = 2048;
  handle_config.conv_frame_size = differential_ ? kSwitchedFrameBytes : kFrameBytes;
  ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_config, &handle_));

  std::array<adc_digi_pattern_config_t, kChannels> pattern{};
  for (std::size_t i = 0; i < kIrCount; ++i) {
    pattern[i].atten = ADC_ATTEN_DB_12;
    pattern[i].channel = pins::kIrAdcChannel[i];
    pattern[i].unit = ADC_UNIT_1;
    pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
  }
  pattern[kIrCount].atten = ADC_ATTEN_DB_12;
  pattern[kIrCount].channel = pins::kBatteryAdcChannel;
  pattern[kIrCount].unit = ADC_UNIT_1;
  pattern[kIrCount].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

  adc_continuous_config_t config{};
  config.pattern_num = kChannels;
  config.adc_pattern = pattern.data();
  config.sample_freq_hz = kSampleFreqHz;
  config.conv_mode = ADC_CONV_SINGLE_UNIT_1;
  ESP_ERROR_CHECK(adc_continuous_config(handle_, &config));
  earliest_us_ = static_cast<std::uint64_t>(esp_timer_get_time());
  ESP_ERROR_CHECK(adc_continuous_start(handle_));
}

bool AdcInputs::drain(std::uint64_t now_us) {
  bool saw_all = false;
  for (int attempt = 0; attempt < 32; ++attempt) {
    alignas(adc_digi_output_data_t) std::array<std::uint8_t, kFrameBytes> bytes{};
    std::uint32_t length = 0;
    const auto status = adc_continuous_read(handle_, bytes.data(), bytes.size(), &length, 0);
    if (status == ESP_ERR_TIMEOUT)
      break;
    if (status != ESP_OK) {
      ++overflows_;
      restart();
      break;
    }
    for (std::size_t offset = 0; offset + sizeof(adc_digi_output_data_t) <= length;
         offset += sizeof(adc_digi_output_data_t)) {
      adc_digi_output_data_t conversion{};
      std::memcpy(&conversion, bytes.data() + offset, sizeof(conversion));
      const int channel = conversion.type2.channel;
      const int expected =
        (next_ < kIrCount) ? pins::kIrAdcChannel[next_] : pins::kBatteryAdcChannel;
      if (channel != expected) {
        next_ = 0;  // resync on the first line channel
        if (channel != pins::kIrAdcChannel[0])
          continue;
      }
      sweep_[next_] = conversion.type2.data;
      ++next_;
      if (next_ == kChannels) {
        next_ = 0;
        saw_all = true;
        battery_sum_ += sweep_[kIrCount];
        ++sweeps_;
        if (settling_sweeps_ > 0) {
          --settling_sweeps_;
          continue;
        }
        for (std::size_t eye = 0; eye < kIrCount; ++eye) {
          ir_sum_[eye] += sweep_[eye];
        }
        ++ir_sweeps_;
      }
    }
  }
  return saw_all;
}

void AdcInputs::restart() {
  next_ = 0;
  ir_sum_.fill(0);
  ir_sweeps_ = 0;
  battery_sum_ = 0;
  sweeps_ = 0;
  ESP_ERROR_CHECK(adc_continuous_stop(handle_));
  ESP_ERROR_CHECK(adc_continuous_flush_pool(handle_));
  ESP_ERROR_CHECK(adc_continuous_start(handle_));
}

void AdcInputs::read(std::uint64_t now_us, Sample<std::array<std::uint16_t, kIrCount>>& ir,
                     Sample<float>& battery) {
  // Every complete sweep since the last tick is averaged, so the control loop
  // sees the mean of about five conversions per eye instead of one. If the ADC
  // stalls nothing is published and the consumer's freshness rule reports it.
  drain(now_us);
  if (sweeps_ == 0 && last_publish_us_ != 0 && now_us - last_publish_us_ > kStallRestartUs &&
      now_us - last_restart_us_ > kRestartHoldoffUs) {
    ++restarts_;
    last_restart_us_ = now_us;
    restart();
  }
  if (sweeps_ == 0) {
    ir.status = SampleStatus::kOverflow;
    battery.status = SampleStatus::kOverflow;
    return;
  }
  const std::uint64_t period =
    (last_publish_us_ != 0 && now_us > last_publish_us_) ? now_us - last_publish_us_ : kPeriodUs;
  const float battery_counts = static_cast<float>(battery_sum_) / static_cast<float>(sweeps_);
  battery.value = battery_counts * 3.3F / 4095.0F * kDefaultDivider;
  publish_ir(ir);
  battery_sum_ = 0;
  sweeps_ = 0;
  ++sequence_;
  ir.earliest_us = (last_publish_us_ != 0) ? last_publish_us_ : now_us;
  ir.latest_us = now_us;
  ir.period_us = static_cast<std::uint32_t>(period);
  ir.sequence = sequence_;
  battery.earliest_us = ir.earliest_us;
  battery.latest_us = now_us;
  battery.period_us = ir.period_us;
  battery.sequence = sequence_;
  battery.status = SampleStatus::kValid;
  last_publish_us_ = now_us;
}

// The mean of this tick's clean sweeps. Switching, it is one half of a lit/unlit
// pair; the published value rises with lit minus unlit and stays in 0..4095. Which
// way the line moves it is the core's calibration to find, as without switching.
void AdcInputs::publish_ir(Sample<std::array<std::uint16_t, kIrCount>>& ir) {
  std::array<std::uint16_t, kIrCount> mean{};
  const std::uint32_t count = ir_sweeps_;
  for (std::size_t eye = 0; eye < kIrCount && count > 0; ++eye) {
    mean[eye] = static_cast<std::uint16_t>((ir_sum_[eye] + count / 2) / count);
  }
  ir_sum_.fill(0);
  ir_sweeps_ = 0;
  if (!differential_) {
    ir.value = mean;
    ir.status = count > 0 ? SampleStatus::kValid : SampleStatus::kOverflow;
    return;
  }
  if (count > 0) {
    (leds_on_ ? lit_ : unlit_) = mean;
    (leds_on_ ? have_lit_ : have_unlit_) = true;
  }
  switch_leds();
  if (!have_lit_ || !have_unlit_) {
    ir.status = SampleStatus::kUnavailable;
    return;
  }
  for (std::size_t eye = 0; eye < kIrCount; ++eye) {
    ir.value[eye] = static_cast<std::uint16_t>((kAdcFull + lit_[eye] - unlit_[eye]) / 2);
  }
  ir.status = SampleStatus::kValid;
}

void AdcInputs::switch_leds() {
  leds_on_ = !leds_on_;
  const bool high = leds_on_ == (pins::kIrCtrlWiring == pins::IrCtrl::kLedOnHigh);
  gpio_set_level(static_cast<gpio_num_t>(pins::kIrCtrl), high ? 1 : 0);
  settling_sweeps_ = kSettlingSweeps;
}

void AdcInputs::probe_ir_ctrl(int level) const {
  if (differential_) {
    return;
  }
  const auto pin = static_cast<gpio_num_t>(pins::kIrCtrl);
  if (level < 0) {
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    return;
  }
  gpio_set_direction(pin, GPIO_MODE_OUTPUT);
  gpio_set_level(pin, level > 0 ? 1 : 0);
}

}  // namespace lf::esp32
