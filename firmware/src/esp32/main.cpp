#include <driver/i2c_master.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <atomic>
#include <cstdint>

#include "adc_inputs.hpp"
#include "blackbox.hpp"
#include "button.hpp"
#include "config.hpp"
#include "console.hpp"
#include "encoders.hpp"
#include "imu.hpp"
#include "motors.hpp"
#include "pins.hpp"
#include "robot.hpp"
#include "status_led.hpp"
#include "tof.hpp"

namespace {

constexpr TickType_t kControlPeriod = pdMS_TO_TICKS(1000 / lf::kControlHz);
constexpr TickType_t kSensorsPeriod = pdMS_TO_TICKS(2);
// ToF finishes a measurement every 25 ms; checking it every 10th pass is plenty.
constexpr unsigned kTofEveryPasses = 10;
// The motors are coasted if the control task misses this many watchdog periods.
constexpr std::uint64_t kWatchdogPeriodUs = 10000;
constexpr unsigned kWatchdogMissLimit = 2;

// Written by the sensors task, read by the control task under the lock.
struct SlowSensors {
  lf::Sample<float> gyro_z{};
  lf::Sample<float> distance{};
  std::uint8_t range_status = 255;
};

// The console posts a request; the control task takes it at the start of a tick.
struct Mailbox {
  lf::Command command;
  std::atomic<bool> pending{false};
  void post(const lf::Command& request) {
    command = request;
    pending.store(true, std::memory_order_release);
  }
  lf::Command take() {
    if (!pending.exchange(false, std::memory_order_acquire)) {
      return {};
    }
    return command;
  }
};

lf::esp32::AdcInputs g_adc;
lf::esp32::Encoders g_encoders;
lf::esp32::Imu g_imu;
lf::esp32::Tof g_tof;
lf::esp32::Motors g_motors;
lf::esp32::Blackbox g_blackbox;
lf::esp32::StatusLed g_led;
lf::esp32::ButtonPin g_button;
lf::Robot g_robot;
Mailbox g_mailbox;
SlowSensors g_slow;
lf::Measurements g_latest;
lf::Snapshot g_snapshot;
lf::esp32::Diagnostics g_diagnostics;
portMUX_TYPE g_lock = portMUX_INITIALIZER_UNLOCKED;
std::atomic<std::uint32_t> g_heartbeat{0};
std::atomic<std::uint32_t> g_tick_max_us{0};
std::atomic<std::uint32_t> g_late_ticks{0};
std::atomic<std::uint32_t> g_watchdog_cuts{0};

std::uint64_t now_us() {
  return static_cast<std::uint64_t>(esp_timer_get_time());
}

// Gyro at 500 Hz; ToF only when its measurement is due. Neither may ever make the
// control task wait, which is why they live here and publish copies.
void sensors_task(void*) {
  TickType_t wake = xTaskGetTickCount();
  unsigned pass = 0;
  // The ToF keeps its last result between reads; an invalid one is published as
  // invalid, never left looking like the previous good distance.
  lf::Sample<float> distance;
  std::uint8_t range_status = 255;
  for (;;) {
    lf::Sample<float> gyro_z;
    g_imu.read(now_us(), gyro_z);
    const bool tof_due = ++pass % kTofEveryPasses == 0;
    if (tof_due) {
      g_tof.read(now_us(), distance, range_status);
    }
    portENTER_CRITICAL(&g_lock);
    if (gyro_z.status == lf::SampleStatus::kValid) {
      g_slow.gyro_z = gyro_z;
    }
    if (tof_due) {
      g_slow.distance = distance;
      g_slow.range_status = range_status;
    }
    portEXIT_CRITICAL(&g_lock);
    vTaskDelayUntil(&wake, kSensorsPeriod);
  }
}

void control_task(void*) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    const std::uint64_t start = now_us();
    lf::Measurements m;
    g_adc.read(start, m.ir, m.battery_v);
    g_encoders.read(start, m.encoders);
    portENTER_CRITICAL(&g_lock);
    m.gyro_z_rad_s = g_slow.gyro_z;
    m.distance_mm = g_slow.distance;
    m.range_status = g_slow.range_status;
    portEXIT_CRITICAL(&g_lock);
    m.adc_overflows = g_adc.overflows();

    const bool button = g_button.down();
    const lf::Outputs outputs = g_robot.step(m, button, start, g_mailbox.take());
    g_motors.apply(outputs.motor);
    g_led.set(outputs.led);

    const lf::Snapshot snapshot = g_robot.snapshot();
    portENTER_CRITICAL(&g_lock);
    g_latest = m;
    g_snapshot = snapshot;
    g_diagnostics.button_down = button;
    portEXIT_CRITICAL(&g_lock);

    g_heartbeat.fetch_add(1, std::memory_order_relaxed);
    const auto spent = static_cast<std::uint32_t>(now_us() - start);
    if (spent > g_tick_max_us.load(std::memory_order_relaxed)) {
      g_tick_max_us.store(spent, std::memory_order_relaxed);
    }
    if (xTaskDelayUntil(&wake, kControlPeriod) == pdFALSE) {
      g_late_ticks.fetch_add(1, std::memory_order_relaxed);
    }
  }
}

void service_task(void*) {
  lf::esp32::console_begin();
  bool was_moving = false;
  for (;;) {
    g_led.render(now_us());
    lf::Measurements copy;
    lf::Snapshot snapshot;
    lf::esp32::Diagnostics diagnostics;
    portENTER_CRITICAL(&g_lock);
    copy = g_latest;
    snapshot = g_snapshot;
    diagnostics = g_diagnostics;
    portEXIT_CRITICAL(&g_lock);
    diagnostics.tick_max_us = g_tick_max_us.load(std::memory_order_relaxed);
    diagnostics.late_ticks = g_late_ticks.load(std::memory_order_relaxed);
    diagnostics.watchdog_cuts = g_watchdog_cuts.load(std::memory_order_relaxed);
    diagnostics.motor_applies = g_motors.apply_count();
    diagnostics.adc_overflows = copy.adc_overflows;
    // When a run ends, keep its log in flash. Flash writes stall the other core, so
    // this happens only after the robot has stopped, and nothing else that can
    // start it (the console) runs in this task meanwhile.
    const bool moving = snapshot.routine_active || snapshot.phase == lf::Phase::kSettling ||
                        snapshot.phase == lf::Phase::kRunning;
    if (was_moving && !moving) {
      static_cast<void>(g_blackbox.save(g_robot.log()));
    }
    was_moving = moving;
    const lf::Command command =
      lf::esp32::console_service(snapshot, copy, g_robot.log(), g_blackbox, diagnostics);
    if (command.kind != lf::Command::Kind::kNone) {
      g_mailbox.post(command);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// Independent of every task: if the control task stops ticking, the motors coast.
void watchdog_callback(void*) {
  static std::uint32_t last_heartbeat = 0;
  static unsigned misses = 0;
  const std::uint32_t heartbeat = g_heartbeat.load(std::memory_order_relaxed);
  if (heartbeat != last_heartbeat) {
    last_heartbeat = heartbeat;
    misses = 0;
    return;
  }
  if (++misses == kWatchdogMissLimit) {
    g_motors.stop();
    g_watchdog_cuts.fetch_add(1, std::memory_order_relaxed);
  }
}

}  // namespace

extern "C" void app_main() {
  g_led.initialize();
  g_led.set({lf::Colour::kOrange, lf::Pattern::kSteady});
  g_button.initialize();
  g_adc.initialize();
  g_encoders.initialize();
  g_motors.initialize();
  g_motors.stop();
  g_blackbox.initialize();

  i2c_master_bus_config_t bus{};
  bus.i2c_port = I2C_NUM_0;
  bus.sda_io_num = static_cast<gpio_num_t>(lf::esp32::pins::kI2cSda);
  bus.scl_io_num = static_cast<gpio_num_t>(lf::esp32::pins::kI2cScl);
  bus.clk_source = I2C_CLK_SRC_DEFAULT;
  bus.glitch_ignore_cnt = 7;
  i2c_master_bus_handle_t bus_handle = nullptr;
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus, &bus_handle));
  g_imu.initialize(bus_handle);
  g_tof.initialize(bus_handle);
  g_diagnostics.imu_identity = g_imu.identity();
  g_diagnostics.tof_identity = g_tof.identity();
  g_diagnostics.imu_ready = g_imu.ready();
  g_diagnostics.tof_ready = g_tof.ready();

  xTaskCreatePinnedToCore(sensors_task, "sensors", 4096, nullptr, 4, nullptr, 0);
  xTaskCreatePinnedToCore(control_task, "control", 6144, nullptr, 6, nullptr, 1);
  xTaskCreatePinnedToCore(service_task, "service", 6144, nullptr, 2, nullptr, 0);

  const esp_timer_create_args_t watchdog_args = {
    .callback = watchdog_callback,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "motor_deadline",
    .skip_unhandled_events = true,
  };
  esp_timer_handle_t watchdog = nullptr;
  ESP_ERROR_CHECK(esp_timer_create(&watchdog_args, &watchdog));
  ESP_ERROR_CHECK(esp_timer_start_periodic(watchdog, kWatchdogPeriodUs));
}
