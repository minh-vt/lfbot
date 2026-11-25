#pragma once

#include <esp_adc/adc_continuous.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "types.hpp"

namespace lf::esp32 {

// One ADC1 DMA sequence carries the eight line sensors and the pack voltage.
// They must share a unit: an ADC cannot run oneshot and continuous together.
//
// With the bar's LEDs on IR_CTRL (pins::kIrCtrlWiring), each tick reads the eyes with
// the LEDs in one state and switches them for the next; what is published is the
// difference between the latest lit and unlit readings, so light that is there either
// way (sun, room lamps) cancels.
class AdcInputs {
public:
  void initialize();
  void read(std::uint64_t now_us, Sample<std::array<std::uint16_t, kIrCount>>& ir,
            Sample<float>& battery);
  // Bench only, and only while IR_CTRL is not in use: drive the bar's IR pin high (1)
  // or low (0), or let it go (-1), to find out what it does.
  void probe_ir_ctrl(int level) const;

private:
  static constexpr std::size_t kChannels = kIrCount + 1;
  bool drain(std::uint64_t now_us);
  void restart();
  void switch_leds();
  void publish_ir(Sample<std::array<std::uint16_t, kIrCount>>& ir);

public:
  [[nodiscard]] std::uint32_t overflows() const { return overflows_; }
  [[nodiscard]] std::uint32_t restarts() const { return restarts_; }

private:
  adc_continuous_handle_t handle_ = nullptr;
  std::array<std::uint16_t, kChannels> sweep_{};
  std::array<std::uint32_t, kIrCount> ir_sum_{};
  std::uint32_t ir_sweeps_ = 0;
  std::uint32_t battery_sum_ = 0;
  std::uint32_t sweeps_ = 0;
  std::size_t next_ = 0;
  std::uint64_t earliest_us_ = 0;
  std::uint64_t last_publish_us_ = 0;
  std::uint32_t overflows_ = 0;
  std::uint32_t restarts_ = 0;
  std::uint64_t last_restart_us_ = 0;
  std::uint32_t sequence_ = 0;

  bool differential_ = false;
  bool leds_on_ = false;
  std::uint32_t settling_sweeps_ = 0;  // sweeps still to skip since the LEDs switched
  std::array<std::uint16_t, kIrCount> lit_{};
  std::array<std::uint16_t, kIrCount> unlit_{};
  bool have_lit_ = false;
  bool have_unlit_ = false;
};

}  // namespace lf::esp32
