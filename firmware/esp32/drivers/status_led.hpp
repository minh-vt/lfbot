#pragma once

#include <driver/rmt_tx.h>

#include <cstdint>

#include "types.hpp"

namespace lf::esp32 {

// Single WS2812 on the board. The operator's only feedback channel at the venue.
class StatusLed {
public:
  void initialize();
  void set(Colour colour);
  // Called from a slow task: sends the colour if it changed.
  void render();

private:
  void show(std::uint8_t red, std::uint8_t green, std::uint8_t blue);

  rmt_channel_handle_t channel_ = nullptr;
  rmt_encoder_handle_t encoder_ = nullptr;
  Colour colour_ = Colour::kOff;
  std::uint32_t last_bits_ = 0xFFFFFFFFU;
};

}  // namespace lf::esp32
