#include "adc_inputs.hpp"

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
constexpr std::uint32_t kPeriodUs = 1000000U / (kSampleFreqHz / (kIrCount + 1));
// Battery divider installed by the owner: a ~1:5 module. The real ratio is a
// robot-profile value; this is only the starting point until it is measured.
constexpr float kDefaultDivider = 5.0F;
// No complete sweep for this long means the DMA has stopped (it ran dry while the
// CPU was busy or the cache was off): start it again. Normal gap is 0.4 ms.
constexpr std::uint64_t kStallRestartUs = 5000;
constexpr std::uint64_t kRestartHoldoffUs = 20000;

}  // namespace

void AdcInputs::initialize() {
  adc_continuous_handle_cfg_t handle_config{};
  handle_config.max_store_buf_size = 2048;
  handle_config.conv_frame_size = kFrameBytes;
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
      last_channel_ = static_cast<std::uint32_t>(channel);
      last_data_ = conversion.type2.data;
      ++conversions_;
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
        for (std::size_t channel_index = 0; channel_index < kChannels; ++channel_index) {
          sum_[channel_index] += sweep_[channel_index];
        }
        ++sweeps_;
      }
    }
  }
  return saw_all;
}

void AdcInputs::restart() {
  next_ = 0;
  sum_.fill(0);
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
  const std::uint32_t count = sweeps_;
  const std::uint64_t period =
    (last_publish_us_ != 0 && now_us > last_publish_us_) ? now_us - last_publish_us_ : kPeriodUs;
  for (std::size_t eye = 0; eye < kIrCount; ++eye) {
    ir.value[eye] = static_cast<std::uint16_t>((sum_[eye] + count / 2) / count);
  }
  const float battery_counts = static_cast<float>(sum_[kIrCount]) / static_cast<float>(count);
  battery.value = battery_counts * 3.3F / 4095.0F * kDefaultDivider;
  sum_.fill(0);
  sweeps_ = 0;
  ++sequence_;
  ir.earliest_us = (last_publish_us_ != 0) ? last_publish_us_ : now_us;
  ir.latest_us = now_us;
  ir.period_us = static_cast<std::uint32_t>(period);
  ir.sequence = sequence_;
  ir.status = SampleStatus::kValid;
  battery.earliest_us = ir.earliest_us;
  battery.latest_us = now_us;
  battery.period_us = ir.period_us;
  battery.sequence = sequence_;
  battery.status = SampleStatus::kValid;
  last_publish_us_ = now_us;
}

}  // namespace lf::esp32
