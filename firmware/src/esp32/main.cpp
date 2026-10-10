#include <Arduino.h>
#include <Wire.h>

#include "actuators.hpp"
#include "kernel/plan.hpp"
#include "kernel/step.hpp"
#include "sensors.hpp"

// Pin map (docs/HARDWARE_GUIDE.md)
namespace {
const int kIrPins[8] = {1, 2, 3, 4, 5, 6, 7, 8};
const int kEncL = 15;
const int kEncR = 17;
const int kMotorL1 = 39;
const int kMotorL2 = 40;
const int kMotorR1 = 41;
const int kMotorR2 = 42;
const int kBtnMode = 47;
const int kBtnStart = 48;
constexpr int kSda = 11;
constexpr int kScl = 12;

enum class RunMode : int { kIdle = 0, kRun1 = 1, kRun2 = 2, kDone = 3 };

volatile std::int32_t g_enc_l = 0;
volatile std::int32_t g_enc_r = 0;

void IRAM_ATTR enc_l_isr() {
  ++g_enc_l;
}
void IRAM_ATTR enc_r_isr() {
  ++g_enc_r;
}

void write_motor(int pin_fwd, int pin_rev, float duty) {
  if (duty >= 0.0f) {
    ledcWrite(pin_fwd, static_cast<int>(duty * 255.0f));
    ledcWrite(pin_rev, 0);
  } else {
    ledcWrite(pin_fwd, 0);
    ledcWrite(pin_rev, static_cast<int>(-duty * 255.0f));
  }
}

// Stub: đọc VL53L0X qua I2C. TODO khi có driver thật.
float read_tof_mm(int /*sensor_idx*/) {
  return 300.0f;
}

}  // namespace

void setup() {
  Serial.begin(115200);

  pinMode(kEncL, INPUT_PULLUP);
  pinMode(kEncR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kEncL), enc_l_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kEncR), enc_r_isr, CHANGE);
  pinMode(kBtnMode, INPUT_PULLUP);
  pinMode(kBtnStart, INPUT_PULLUP);

  // MCPWM: 4 kênh LEDC cho 2 cầu H.
  constexpr int kFreq = 20000;
  constexpr int kRes = 8;
  ledcSetup(0, kFreq, kRes);
  ledcAttachPin(kMotorL1, 0);
  ledcSetup(1, kFreq, kRes);
  ledcAttachPin(kMotorL2, 1);
  ledcSetup(2, kFreq, kRes);
  ledcAttachPin(kMotorR1, 2);
  ledcSetup(3, kFreq, kRes);
  ledcAttachPin(kMotorR2, 3);

  Wire.begin(kSda, kScl);
  delay(100);
}

void loop() {
  static RunMode mode = RunMode::kIdle;
  static lf::kernel::KernelState state;
  static lf::kernel::TrackPlan plan;
  static lf::kernel::TuneConfig cfg;
  static unsigned long last = micros();

  const unsigned long now = micros();
  const float dt = constrain((now - last) / 1e6f, 0.0f, 0.01f);
  last = now;

  if (digitalRead(kBtnStart) == LOW && mode == RunMode::kIdle)
    mode = RunMode::kRun1;
  if (digitalRead(kBtnMode) == LOW && mode == RunMode::kDone)
    mode = RunMode::kIdle;

  lf::kernel::SensorFrame frame;
  lf::esp32::IrReadings ir;
  for (int i = 0; i < 8; ++i)
    ir.raw[i] = static_cast<std::uint16_t>(analogRead(kIrPins[i]));
  const auto ir_res = lf::esp32::normalize_ir(ir);
  frame.line_error = ir_res.line_error;
  frame.line_detected = ir_res.line_detected;

  const auto tof = lf::esp32::normalize_tof(read_tof_mm(0), read_tof_mm(1), read_tof_mm(2));
  frame.obstacle_danger = tof.danger;
  frame.obstacle_side = tof.side;

  noInterrupts();
  frame.enc_l = g_enc_l;
  frame.enc_r = g_enc_r;
  interrupts();

  frame.dt = dt;
  frame.finish_detected = false;  // TODO: cảm biến đích (marker/vạch đặc biệt)

  const auto result = lf::kernel::step(state, frame, plan, cfg);
  state = result.state;

  if (mode == RunMode::kRun1) {
    const float tick = 0.5f * (static_cast<float>(frame.enc_l) + static_cast<float>(frame.enc_r));
    lf::kernel::record(plan, tick, result.path_steer, frame.obstacle_side);
  }

  write_motor(kMotorL1, kMotorL2, result.command.left_duty);
  write_motor(kMotorR1, kMotorR2, result.command.right_duty);

  if (mode == RunMode::kRun1 && state.finish_latched) {
    lf::kernel::bake(plan, cfg);
    state = {};
    mode = RunMode::kRun2;
  } else if (mode == RunMode::kRun2 && state.finish_latched) {
    write_motor(kMotorL1, kMotorL2, 0.0f);
    write_motor(kMotorR1, kMotorR2, 0.0f);
    mode = RunMode::kDone;
  }

  delay(2);  // 500Hz loop
}
