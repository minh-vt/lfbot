#pragma once

#include <driver/mcpwm_prelude.h>
#include <freertos/FreeRTOS.h>

#include <array>
#include <cstdint>

#include "types.hpp"

namespace lf::esp32 {

// Sole owner of the motor pins. The control task drives them; the deadline
// watchdog may only coast them, and the lock keeps the two from interleaving.
class Motors {
public:
  void initialize();
  void apply(const MotorOutput& output);
  void stop();
  [[nodiscard]] std::uint32_t apply_count() const { return apply_count_; }

private:
  struct Channel {
    mcpwm_cmpr_handle_t comparator = nullptr;
    mcpwm_gen_handle_t generator = nullptr;
  };
  void set_static(Channel& channel, bool high);
  void set_duty(Channel& channel, float duty);

  std::array<std::array<Channel, 2>, kWheelCount> channels_{};
  mcpwm_timer_handle_t timer_ = nullptr;
  std::uint32_t apply_count_ = 0;
  portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
};

}  // namespace lf::esp32
