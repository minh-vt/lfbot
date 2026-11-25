#pragma once

#include <cstdint>

#include "adc_inputs.hpp"
#include "blackbox.hpp"
#include "robot.hpp"

namespace lf::esp32 {

// What the console may report about the platform, so it cannot reach into drivers.
struct Diagnostics {
  std::uint32_t adc_overflows = 0;
  std::uint32_t adc_restarts = 0;
  std::uint8_t imu_identity = 0;
  std::uint16_t tof_identity = 0;
  bool imu_ready = false;
  bool tof_ready = false;
  std::uint32_t motor_applies = 0;
  std::uint32_t tick_max_us = 0;
  std::uint32_t late_ticks = 0;
  std::uint32_t watchdog_cuts = 0;
  bool button_down = false;
};

// Bench only: it reports copies of what the robot is doing and never moves it. The
// one thing it drives is the bar's IR pin, while the robot is idle, to find out what
// that pin does. The operator at the venue has a button and a LED.
void console_begin();
void console_service(const Snapshot& snapshot, const Measurements& measurements, const LogRing& log,
                     const Blackbox& blackbox, const AdcInputs& adc,
                     const Diagnostics& diagnostics);

}  // namespace lf::esp32
