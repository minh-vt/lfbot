#include <driver/gpio.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdint>

#include "actuators.hpp"
#include "kernel/mission.hpp"
#include "sensors.hpp"

namespace {

constexpr TickType_t kControlPeriod = pdMS_TO_TICKS(2);  // 500Hz (cần tick ≥ 1kHz)
constexpr float kControlDt = 0.002f;
constexpr int kButtonStart = 48;
constexpr int kButtonMode = 47;

// Plan ở BSS (48KB) — không nằm trên stack task.
lf::kernel::TrackPlan g_plan;
lf::kernel::KernelState g_state;
lf::kernel::RunMode g_mode = lf::kernel::RunMode::kIdle;

bool button_pressed(int pin) {
  return gpio_get_level(static_cast<gpio_num_t>(pin)) == 0;
}

// Vòng điều khiển 500Hz hard real-time (FreeRTOS task, core 1).
void control_task(void* /*arg*/) {
  const lf::kernel::TuneConfig cfg;
  TickType_t last_wake = xTaskGetTickCount();

  for (;;) {
    if (button_pressed(kButtonStart) && g_mode == lf::kernel::RunMode::kIdle) {
      g_mode = lf::kernel::RunMode::kRun1;
    }
    if (button_pressed(kButtonMode) && g_mode == lf::kernel::RunMode::kDone) {
      g_mode = lf::kernel::RunMode::kIdle;
      g_plan.reset();
    }

    lf::kernel::SensorFrame frame;
    lf::esp32::sensors_read(frame, kControlDt);
    const lf::kernel::MotorCommand command =
      lf::kernel::mission_step(g_state, frame, g_plan, g_mode, cfg);
    lf::esp32::actuators_write(command);

    const lf::kernel::MissionEvent event =
      lf::kernel::advance_mission(g_plan, g_state, g_mode, cfg);
    if (event == lf::kernel::MissionEvent::kRun2Done) {
      lf::esp32::actuators_write({0.0f, 0.0f, true});
    }

    vTaskDelayUntil(&last_wake, kControlPeriod);
  }
}

}

extern "C" void app_main() {
  gpio_config_t buttons = {};
  buttons.pin_bit_mask = (1ULL << kButtonStart) | (1ULL << kButtonMode);
  buttons.mode = GPIO_MODE_INPUT;
  buttons.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_config(&buttons);

  lf::esp32::sensors_init();
  lf::esp32::actuators_init();

  xTaskCreatePinnedToCore(control_task, "control", 8192, nullptr, 5, nullptr, 1);
}
