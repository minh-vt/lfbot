#pragma once

#include <driver/rmt_tx.h>

#include <cstdint>

#include "types.hpp"

namespace lf::esp32 {

// Single WS2812 on the board. The operator's only feedback channel at the venue.
class StatusLed {
public:
  void initialize();
  void set(const LedPattern& pattern);
  // Called from a slow task: works out what the LED should show right now.
  void render(std::uint64_t now_us);

private:
  void show(std::uint8_t red, std::uint8_t green, std::uint8_t blue);

  rmt_channel_handle_t channel_ = nullptr;
  rmt_encoder_handle_t encoder_ = nullptr;
  LedPattern pattern_{};
  std::uint32_t last_bits_ = 0xFFFFFFFFU;
};

}  // namespace lf::esp32
