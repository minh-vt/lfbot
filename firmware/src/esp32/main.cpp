#include <driver/gpio.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdint>

#include "actuators.hpp"
#include "drive/plan.hpp"
#include "drive/step.hpp"
#include "sensing/layout.hpp"
#include "sensors.hpp"

namespace {

constexpr TickType_t kControlPeriod = pdMS_TO_TICKS(2);  // 500Hz (cần tick ≥ 1kHz)
constexpr float kControlDt = 0.002f;
constexpr int kButtonStart = 48;
constexpr int kButtonMode = 47;

// Plan ở BSS (48KB) — không nằm trên stack task.
lf::drive::RacePlan g_plan;
lf::drive::DriveState g_state;
lf::drive::RunMode g_mode = lf::drive::RunMode::kIdle;

bool button_pressed(int pin) {
  return gpio_get_level(static_cast<gpio_num_t>(pin)) == 0;
}

// Vòng điều khiển 500Hz hard real-time (FreeRTOS task, core 1).
void control_task(void* /*arg*/) {
  lf::drive::Tuning cfg;
  // Hình học không-thứ-nguyên suy trực tiếp từ layout SSoT.
  cfg.robot.geometry_l = lf::sensing::geometry_l(lf::sensing::kRobotLayout);
  cfg.robot.geometry_s = lf::sensing::geometry_s(lf::sensing::kRobotLayout);
  TickType_t last_wake = xTaskGetTickCount();

  for (;;) {
    if (button_pressed(kButtonStart) && g_mode == lf::drive::RunMode::kIdle) {
      g_mode = lf::drive::RunMode::kRun1;
    }
    if (button_pressed(kButtonMode) && g_mode == lf::drive::RunMode::kDone) {
      g_mode = lf::drive::RunMode::kIdle;
      g_plan.reset();
    }

    lf::drive::Readings frame;
    lf::esp32::sensors_read(frame, kControlDt);
    const auto result = lf::drive::step(g_state, frame, g_plan, cfg);
    g_state = result.state;
    if (g_mode == lf::drive::RunMode::kRun1) {
      const float tick = 0.5f * (static_cast<float>(frame.enc_l) + static_cast<float>(frame.enc_r));
      lf::drive::record(g_plan, tick, result.path_steer, frame.obstacle_side);
    }
    lf::esp32::actuators_write(result.command);

    if (g_mode == lf::drive::RunMode::kRun1 && g_state.finish_latched) {
      lf::drive::plan_speed(g_plan, cfg);
      g_state = {};
      g_mode = lf::drive::RunMode::kRun2;
    }
    if (g_mode == lf::drive::RunMode::kRun2 && g_state.finish_latched) {
      lf::esp32::actuators_write({0.0f, 0.0f, true});
      g_mode = lf::drive::RunMode::kDone;
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
