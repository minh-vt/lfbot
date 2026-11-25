#include "encoders.hpp"

#include <esp_err.h>

#include "pins.hpp"

namespace lf::esp32 {
namespace {

pcnt_unit_handle_t make_quadrature(int a, int b) {
  pcnt_unit_config_t config{};
  config.low_limit = -30000;
  config.high_limit = 30000;
  config.flags.accum_count = 1;
  pcnt_unit_handle_t unit = nullptr;
  ESP_ERROR_CHECK(pcnt_new_unit(&config, &unit));
  const std::array<int, 2> pins = {a, b};
  for (std::size_t n = 0; n < pins.size(); ++n) {
    pcnt_chan_config_t channel_config{};
    channel_config.edge_gpio_num = pins[n];
    channel_config.level_gpio_num = pins[1 - n];
    pcnt_channel_handle_t channel = nullptr;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &channel_config, &channel));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(channel, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
      channel, n == 0 ? PCNT_CHANNEL_LEVEL_ACTION_KEEP : PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
      n == 0 ? PCNT_CHANNEL_LEVEL_ACTION_INVERSE : PCNT_CHANNEL_LEVEL_ACTION_KEEP));
  }
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, config.low_limit));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, config.high_limit));
  ESP_ERROR_CHECK(pcnt_unit_enable(unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
  ESP_ERROR_CHECK(pcnt_unit_start(unit));
  return unit;
}

}  // namespace

void Encoders::initialize() {
  units_[0] = make_quadrature(pins::kEncoderLeftA, pins::kEncoderLeftB);
  units_[1] = make_quadrature(pins::kEncoderRightA, pins::kEncoderRightB);
}

void Encoders::read(std::uint64_t now_us, Sample<std::array<std::int32_t, kWheelCount>>& sample) {
  sample.status = SampleStatus::kValid;
  for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
    int count = 0;
    if (pcnt_unit_get_count(units_[wheel], &count) != ESP_OK) {
      sample.status = SampleStatus::kInvalid;
      count = 0;
    }
    sample.value[wheel] = count;
  }
  ++sample.sequence;
  sample.earliest_us = now_us;
  sample.latest_us = now_us;
  sample.period_us = kPeriodUs;
}

}  // namespace lf::esp32
