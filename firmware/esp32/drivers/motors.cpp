#include "motors.hpp"

#include <esp_err.h>

#include <algorithm>
#include <cmath>

#include "config.hpp"
#include "pins.hpp"

namespace lf::esp32 {
namespace {

constexpr std::uint32_t kTimerResolutionHz = 10000000;  // 0.1 us per tick
constexpr std::uint32_t kPwmPeriodTicks = 500;          // 20 kHz, 0.2% duty steps

}  // namespace

void Motors::initialize() {
  mcpwm_timer_config_t timer_config{};
  timer_config.group_id = 0;
  timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  timer_config.resolution_hz = kTimerResolutionHz;
  timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
  timer_config.period_ticks = kPwmPeriodTicks;
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer_));

  const std::array<std::array<int, 2>, kWheelCount> pins = {{
    {pins::kMotorLeftIn1, pins::kMotorLeftIn2},
    {pins::kMotorRightIn1, pins::kMotorRightIn2},
  }};
  for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
    mcpwm_operator_config_t operator_config{};
    operator_config.group_id = 0;
    mcpwm_oper_handle_t oper = nullptr;
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer_));
    for (std::size_t side = 0; side < 2; ++side) {
      mcpwm_comparator_config_t comparator_config{};
      comparator_config.flags.update_cmp_on_tez = true;
      ESP_ERROR_CHECK(
        mcpwm_new_comparator(oper, &comparator_config, &channels_[wheel][side].comparator));
      mcpwm_generator_config_t generator_config{};
      generator_config.gen_gpio_num = pins[wheel][side];
      ESP_ERROR_CHECK(
        mcpwm_new_generator(oper, &generator_config, &channels_[wheel][side].generator));
      ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        channels_[wheel][side].generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH)));
      ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        channels_[wheel][side].generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, channels_[wheel][side].comparator,
                                       MCPWM_GEN_ACTION_LOW)));
      set_static(channels_[wheel][side], false);
    }
  }
  ESP_ERROR_CHECK(mcpwm_timer_enable(timer_));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer_, MCPWM_TIMER_START_NO_STOP));
}

void Motors::set_static(Channel& channel, bool high) {
  ESP_ERROR_CHECK(mcpwm_generator_set_force_level(channel.generator, high ? 1 : 0, true));
}

void Motors::set_duty(Channel& channel, float duty) {
  // set_static used hold_on=true, which engages the *continue* force. The release
  // must target the same mechanism: level -1 with hold_on=true disables it.
  ESP_ERROR_CHECK(mcpwm_generator_set_force_level(channel.generator, -1, true));
  const auto compare =
    static_cast<std::uint32_t>(std::clamp(duty, 0.0F, 1.0F) * static_cast<float>(kPwmPeriodTicks));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(channel.comparator, compare));
}

void Motors::stop() {
  MotorOutput coast;
  coast.mode = MotorMode::kCoast;
  apply(coast);
}

void Motors::apply(const MotorOutput& output) {
  portENTER_CRITICAL(&lock_);
  ++apply_count_;
  for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
    Channel& first = channels_[wheel][0];
    Channel& second = channels_[wheel][1];
    switch (output.mode) {
      case MotorMode::kCoast:
        set_static(first, false);
        set_static(second, false);
        break;
      case MotorMode::kBrake:
        set_static(first, true);
        set_static(second, true);
        break;
      case MotorMode::kDrive: {
        const float duty = std::clamp(output.duty[wheel], -1.0F, 1.0F);
        if (duty >= 0.0F) {
          set_duty(first, duty);
          set_static(second, false);
        } else {
          set_duty(second, -duty);
          set_static(first, false);
        }
        break;
      }
    }
  }
  portEXIT_CRITICAL(&lock_);
}

}  // namespace lf::esp32
