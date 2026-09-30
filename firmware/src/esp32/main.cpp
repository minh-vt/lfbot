#ifdef PLATFORM_ESP32

#include <Arduino.h>

#include "system/robot_coordinator.hpp"

// Khởi tạo thực thể RobotCoordinator (SSoT)
static lf::RobotConfig g_config{};
static lf::RobotCoordinator g_robot(g_config);

// Biến chia sẻ giữa Core 0 và Core 1 (Thread-safe)
static volatile float g_tof_front_m = 1.0f;
static volatile float g_tof_side_m = 1.0f;
static volatile float g_battery_v = 8.0f;

// TASK CORE 1: ĐIỀU KHIỂN CHUYỂN ĐỘNG THỜI GIAN THỰC (500Hz - Hard Realtime)
void Core1_MotionControlTask(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(2);  // Chu kỳ cứng 2ms = 500Hz

  lf::SensorData in{};
  in.dt = 0.002f;

  while (true) {
    // 1. Đọc dữ liệu cảm biến hồng ngoại qua ADC1 (GPIO 1-8)
    for (size_t i = 0; i < lf::LineSensorConfig::kSensorCount; ++i) {
      in.ir_raw.at(i) = static_cast<float>(analogRead(static_cast<int>(i) + 1)) / 4095.0f;
    }

    // 2. Lấy dữ liệu ToF từ Core 0
    in.tof_front_m = g_tof_front_m;
    in.tof_side_m = g_tof_side_m;

    // 3. Đọc thanh ghi phần cứng PCNT (Pulse Counter)
    // pcnt_unit_get_count(pcnt_unit_l, &in.enc_left_ticks);
    // pcnt_unit_get_count(pcnt_unit_r, &in.enc_right_ticks);

    // 4. Gọi bộ điều phối trung tâm (SSoT)
    lf::MotorCmd out{};
    g_robot.update(in, out);

    // 5. Xuất xung PWM tới MCPWM (IN1, IN2 của A4950)
    // (Băm xung PWM hoặc kích hoạt active_brake)

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// TASK CORE 0: I2C ToF, MÀN HÌNH OLED & NÚT NHẤN (50Hz)
void Core0_InterfaceTask(void* pvParameters) {
  while (true) {
    // 1. Quét cảm biến khoảng cách Laser ToF qua I2C (50Hz)
    // g_tof_front_m = tof_front.readRangeContinuousMillimeters() * 0.001f;
    // g_tof_side_m = tof_side.readRangeContinuousMillimeters() * 0.001f;

    // 2. Đọc điện áp pin từ cầu phân áp (GPIO 9)
    // g_battery_v = analogRead(9) * (3.3f / 4095.0f) * ((100.0f + 33.0f) / 33.0f);

    // 3. Xử lý nút bấm vật lý:
    // - Nhấn BTN_START: Chuyển đổi giữa RUN_1_MAP và RUN_2_SPRINT
    // - Nhấn giữ BTN_MODE: g_robot.start_auto_tune();

    vTaskDelay(pdMS_TO_TICKS(20));  // Chu kỳ 20ms = 50Hz
  }
}

void setup() {
  Serial.begin(115200);
  g_robot.init();

  xTaskCreatePinnedToCore(Core1_MotionControlTask, "MotionTask", 4096, NULL,
                          configMAX_PRIORITIES - 1, NULL, 1);

  xTaskCreatePinnedToCore(Core0_InterfaceTask, "InterfaceTask", 4096, NULL, 5, NULL, 0);
}

void loop() {
  vTaskDelete(NULL);
}

#endif  // PLATFORM_ESP32
