#pragma once

#include <esp_adc/adc_continuous.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "types.hpp"

namespace lf::esp32 {

// One ADC1 DMA sequence carries the eight line sensors and the pack voltage.
// They must share a unit: an ADC cannot run oneshot and continuous together.
class AdcInputs {
public:
  void initialize();
  void read(std::uint64_t now_us, Sample<std::array<std::uint16_t, kIrCount>>& ir,
            Sample<float>& battery);

private:
  static constexpr std::size_t kChannels = kIrCount + 1;
  bool drain(std::uint64_t now_us);

public:
  [[nodiscard]] std::uint32_t last_channel() const { return last_channel_; }
  [[nodiscard]] std::uint32_t last_data() const { return last_data_; }
  [[nodiscard]] std::uint32_t conversions() const { return conversions_; }
  [[nodiscard]] std::uint32_t overflows() const { return overflows_; }

private:
  adc_continuous_handle_t handle_ = nullptr;
  std::uint32_t last_channel_ = 999;
  std::uint32_t last_data_ = 0;
  std::uint32_t conversions_ = 0;
  std::array<std::uint16_t, kChannels> sweep_{};
  std::array<std::uint32_t, kChannels> sum_{};
  std::uint32_t sweeps_ = 0;
  std::size_t next_ = 0;
  std::uint64_t earliest_us_ = 0;
  std::uint64_t last_publish_us_ = 0;
  std::uint32_t overflows_ = 0;
  std::uint32_t sequence_ = 0;
};

}  // namespace lf::esp32
