#include "actuators.hpp"

#include <driver/mcpwm_prelude.h>
#include <esp_err.h>

#include <algorithm>
#include <cmath>

namespace lf::esp32 {
namespace {

constexpr uint32_t kTimerResolutionHz = 1000000;  // 1 MHz
constexpr uint32_t kPeriodTicks = 50;             // 1 MHz / 50 = 20 kHz
constexpr int kLeftPwm = 39, kLeftDir = 40;
constexpr int kRightPwm = 41, kRightDir = 42;
constexpr int kGroup = 0;

struct Motor {
  mcpwm_timer_handle_t timer = nullptr;
  mcpwm_cmpr_handle_t cmp_pwm = nullptr;
  mcpwm_cmpr_handle_t cmp_dir = nullptr;
};

Motor g_left;
Motor g_right;

void init_motor(int gpio_pwm, int gpio_dir, Motor& motor) {
  mcpwm_timer_config_t timer_cfg = {};
  timer_cfg.group_id = kGroup;
  timer_cfg.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  timer_cfg.resolution_hz = kTimerResolutionHz;
  timer_cfg.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
  timer_cfg.period_ticks = kPeriodTicks;
  mcpwm_new_timer(&timer_cfg, &motor.timer);

  mcpwm_oper_handle_t oper = nullptr;
  mcpwm_operator_config_t oper_cfg = {};
  oper_cfg.group_id = kGroup;
  mcpwm_new_operator(&oper_cfg, &oper);
  mcpwm_operator_connect_timer(oper, motor.timer);

  mcpwm_comparator_config_t cmp_cfg = {};
  cmp_cfg.flags.update_cmp_on_tez = true;

  mcpwm_generator_config_t gen_cfg = {};

  gen_cfg.gen_gpio_num = gpio_pwm;
  mcpwm_gen_handle_t gen_pwm = nullptr;
  mcpwm_new_generator(oper, &gen_cfg, &gen_pwm);
  mcpwm_new_comparator(oper, &cmp_cfg, &motor.cmp_pwm);
  mcpwm_generator_set_action_on_timer_event(
    gen_pwm, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
                                          MCPWM_GEN_ACTION_HIGH));
  mcpwm_generator_set_action_on_compare_event(
    gen_pwm,
    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor.cmp_pwm, MCPWM_GEN_ACTION_LOW));

  gen_cfg.gen_gpio_num = gpio_dir;
  mcpwm_gen_handle_t gen_dir = nullptr;
  mcpwm_new_generator(oper, &gen_cfg, &gen_dir);
  mcpwm_new_comparator(oper, &cmp_cfg, &motor.cmp_dir);
  mcpwm_generator_set_action_on_timer_event(
    gen_dir, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
                                          MCPWM_GEN_ACTION_HIGH));
  mcpwm_generator_set_action_on_compare_event(
    gen_dir,
    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor.cmp_dir, MCPWM_GEN_ACTION_LOW));

  mcpwm_timer_enable(motor.timer);
  mcpwm_timer_start_stop(motor.timer, MCPWM_TIMER_START_NO_STOP);
}

// duty ∈ [-1,1]: dương → PWM ở kênh pwm (dir=0); âm → ngược lại.
void set_motor(const Motor& motor, float duty) {
  const float magnitude = std::min(1.0f, std::abs(duty));
  const auto ticks = static_cast<uint32_t>(magnitude * static_cast<float>(kPeriodTicks));
  if (duty >= 0.0f) {
    mcpwm_comparator_set_compare_value(motor.cmp_pwm, ticks);
    mcpwm_comparator_set_compare_value(motor.cmp_dir, 0);
  } else {
    mcpwm_comparator_set_compare_value(motor.cmp_pwm, 0);
    mcpwm_comparator_set_compare_value(motor.cmp_dir, ticks);
  }
}

}  // namespace

void actuators_init() {
  init_motor(kLeftPwm, kLeftDir, g_left);
  init_motor(kRightPwm, kRightDir, g_right);
}

void actuators_write(const lf::kernel::MotorCommand& cmd) {
  if (cmd.brake) {
    set_motor(g_left, 0.0f);
    set_motor(g_right, 0.0f);
    return;
  }
  set_motor(g_left, cmd.left_duty);
  set_motor(g_right, cmd.right_duty);
}

}  // namespace lf::esp32
