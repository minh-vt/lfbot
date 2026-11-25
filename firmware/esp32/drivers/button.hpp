#pragma once

#include <driver/gpio.h>

#include "pins.hpp"

namespace lf::esp32 {

// Raw button level only. Debounce and gestures are core logic, not hardware.
class ButtonPin {
public:
  void initialize() {
    gpio_config_t config{};
    config.pin_bit_mask = 1ULL << pins::kButton;
    config.mode = GPIO_MODE_INPUT;
    config.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&config));
  }
  [[nodiscard]] bool down() const {
    return gpio_get_level(static_cast<gpio_num_t>(pins::kButton)) == 0;
  }
};

}  // namespace lf::esp32
