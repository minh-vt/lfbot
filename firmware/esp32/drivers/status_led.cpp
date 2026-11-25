#include "status_led.hpp"

#include <esp_err.h>

#include <array>

#include "pins.hpp"

namespace lf::esp32 {
namespace {

constexpr std::uint32_t kResolutionHz = 10000000;  // 0.1 us per tick
constexpr std::uint16_t kBit0High = 3;             // 0.3 us
constexpr std::uint16_t kBit0Low = 9;              // 0.9 us
constexpr std::uint16_t kBit1High = 9;
constexpr std::uint16_t kBit1Low = 3;
constexpr std::uint16_t kResetTicks = 300;  // 30 us low, twice, meets the >50 us latch time
constexpr unsigned kLevel = 40;  // out of 255: next to the operator's eyes, never full power

struct Rgb {
  std::uint8_t r;
  std::uint8_t g;
  std::uint8_t b;
};

// Full-brightness hue; scaled down before it is sent.
Rgb hue_of(Colour colour) {
  switch (colour) {
    case Colour::kRed:
      return {255, 0, 0};
    case Colour::kOrange:
      return {255, 70, 0};
    case Colour::kYellow:
      return {255, 190, 0};
    case Colour::kGreen:
      return {0, 255, 0};
    case Colour::kCyan:
      return {0, 220, 220};
    case Colour::kBlue:
      return {0, 0, 255};
    case Colour::kPurple:
      return {130, 0, 255};
    case Colour::kMagenta:
      return {255, 0, 160};
    case Colour::kWhite:
      return {200, 200, 200};
    case Colour::kOff:
      return {0, 0, 0};
  }
  return {0, 0, 0};
}

std::uint8_t scaled(std::uint8_t value, unsigned level) {
  return static_cast<std::uint8_t>((static_cast<unsigned>(value) * level) / 255U);
}

}  // namespace

void StatusLed::initialize() {
  rmt_tx_channel_config_t config{};
  config.gpio_num = static_cast<gpio_num_t>(pins::kStatusLed);
  config.clk_src = RMT_CLK_SRC_DEFAULT;
  config.resolution_hz = kResolutionHz;
  config.mem_block_symbols = 64;
  config.trans_queue_depth = 4;
  ESP_ERROR_CHECK(rmt_new_tx_channel(&config, &channel_));
  rmt_copy_encoder_config_t encoder_config{};
  ESP_ERROR_CHECK(rmt_new_copy_encoder(&encoder_config, &encoder_));
  ESP_ERROR_CHECK(rmt_enable(channel_));
}

void StatusLed::set(Colour colour) {
  colour_ = colour;
}

void StatusLed::render() {
  const Rgb hue = hue_of(colour_);
  show(scaled(hue.r, kLevel), scaled(hue.g, kLevel), scaled(hue.b, kLevel));
}

void StatusLed::show(std::uint8_t red, std::uint8_t green, std::uint8_t blue) {
  // WS2812 expects the bytes in G, R, B order, most significant bit first.
  const std::uint32_t bits =
    (static_cast<std::uint32_t>(green) << 16U) | (static_cast<std::uint32_t>(red) << 8U) | blue;
  if (bits == last_bits_) {
    return;
  }
  last_bits_ = bits;

  std::array<rmt_symbol_word_t, 26> symbols{};
  std::size_t index = 0;
  for (int bit = 23; bit >= 0; --bit) {
    const bool one = ((bits >> static_cast<unsigned>(bit)) & 1U) != 0;
    symbols[index].level0 = 1;
    symbols[index].duration0 = one ? kBit1High : kBit0High;
    symbols[index].level1 = 0;
    symbols[index].duration1 = one ? kBit1Low : kBit0Low;
    ++index;
  }
  symbols[index].level0 = 0;
  symbols[index].duration0 = kResetTicks;
  symbols[index].level1 = 0;
  symbols[index].duration1 = kResetTicks;
  ++index;

  rmt_transmit_config_t transmit{};
  transmit.loop_count = 0;
  transmit.flags.eot_level = 0;
  transmit.flags.queue_nonblocking = 0;
  static_cast<void>(
    rmt_transmit(channel_, encoder_, symbols.data(), index * sizeof(rmt_symbol_word_t), &transmit));
}

}  // namespace lf::esp32
