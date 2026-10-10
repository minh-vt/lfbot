#include <cmath>
#include <cstdio>

#include "kernel/plan.hpp"
#include "kernel/step.hpp"
#include "kernel/tuning.hpp"
#include "sensors.hpp"

namespace {

int& failures() {
  static int count = 0;
  return count;
}

void check(bool condition, const char* expression, int line) {
  if (!condition) {
    (void)std::printf("FAIL line %d: %s\n", line, expression);
    ++failures();
  }
}

#define CHECK(cond) check((cond), #cond, __LINE__)

bool near(float a, float b, float eps = 1e-3f) {
  return std::abs(a - b) < eps;
}

void test_line_error() {
  using namespace lf::kernel;
  SensorFrame frame;
  frame.line_detected = true;
  frame.line_error = 0.5f;
  KernelState state;
  CHECK(near(resolve_line_error(frame, state), 0.5f));
  frame.line_detected = false;
  state.last_direction = -1.0f;
  CHECK(near(resolve_line_error(frame, state), -1.0f));
}

void test_steer() {
  using namespace lf::kernel;
  const TuneConfig cfg;
  KernelState state;
  state.prev_error = 0.1f;
  CHECK(near(steer_pd(0.1f, 0.01f, state, cfg), cfg.robot.steer_kp * 0.1f, 1e-2f));
  state.prev_error = 0.0f;
  CHECK(steer_pd(0.2f, 0.01f, state, cfg) > 0.0f);  // lệch phải → lái dương
}

void test_evasion() {
  using namespace lf::kernel;
  SensorFrame frame;
  frame.obstacle_side = -1.0f;  // vật cản trái
  frame.obstacle_danger = 1.0f;
  const TuneConfig cfg;
  CHECK(evasion_bias(frame, cfg) > 0.0f);  // rẽ phải
}

void test_mix() {
  using namespace lf::kernel;
  const auto straight = mix_differential(1.0f, 0.0f, false);
  CHECK(near(straight.left_duty, 1.0f) && near(straight.right_duty, 1.0f));
  const auto stopped = mix_differential(0.5f, 0.0f, true);
  CHECK(stopped.brake && near(stopped.left_duty, 0.0f) && near(stopped.right_duty, 0.0f));
}

void test_step_run1() {
  using namespace lf::kernel;
  SensorFrame frame;
  frame.line_detected = true;
  frame.line_error = 0.0f;
  frame.dt = 0.01f;
  const KernelState state;
  const TrackPlan plan;
  const TuneConfig cfg;
  const auto result = step(state, frame, plan, cfg);
  CHECK(!result.command.brake);
  CHECK(near(result.command.left_duty, result.command.right_duty, 1e-2f));
  CHECK(result.command.left_duty > 0.0f);
}

void test_step_finish() {
  using namespace lf::kernel;
  SensorFrame frame;
  frame.line_detected = true;
  frame.finish_detected = true;
  const KernelState state;
  const TrackPlan plan;
  const TuneConfig cfg;
  const auto result = step(state, frame, plan, cfg);
  CHECK(result.state.finish_latched);
  CHECK(result.command.brake);
}

void test_bake() {
  using namespace lf::kernel;
  TrackPlan plan;
  record(plan, 0.0f, 0.0f, 0.0f);
  record(plan, 1.0f, 0.0f, 0.0f);
  record(plan, 2.0f, 1.0f, 0.0f);  // cua tại index 2
  TuneConfig cfg;
  cfg.robot.corner_slowdown = 0.6f;
  cfg.track.decel_per_tick = 0.2f;
  bake(plan, cfg);
  CHECK(plan.length == 3);
  CHECK(plan.baked);
  CHECK(plan.throttle_max[2] < plan.throttle_max[1]);
  CHECK(plan.throttle_max[1] < plan.throttle_max[0]);  // giảm dần trước cua
  CHECK(near(plan.throttle_max[2], 0.4f));
}

void test_normalize_ir() {
  lf::esp32::IrReadings readings;
  for (auto& value : readings.raw) {
    value = 4095;  // toàn trắng
  }
  auto result = lf::esp32::normalize_ir(readings);
  CHECK(!result.line_detected);
  readings.raw[7] = 0;  // vạch ở cảm biến phải ngoài
  result = lf::esp32::normalize_ir(readings);
  CHECK(result.line_detected);
  CHECK(result.line_error > 0.9f);
}

void test_normalize_tof() {
  lf::kernel::SensorFrame frame;
  lf::kernel::normalize_tof(frame, 300.0f, 300.0f, 100.0f, 300.0f);
  CHECK(frame.obstacle_danger > 0.0f);
  CHECK(frame.obstacle_side < 0.0f);  // vật cản trái
  lf::kernel::normalize_tof(frame, 300.0f, 300.0f, 300.0f, 300.0f);
  CHECK(near(frame.obstacle_danger, 0.0f) && near(frame.obstacle_side, 0.0f));
}

void test_tuning_derive() {
  using namespace lf::kernel;
  // Lọc đạo hàm: tần số cắt cao hơn → alpha lớn hơn.
  CHECK(derive_d_filter_alpha(0.002f, 50.0f) > derive_d_filter_alpha(0.002f, 10.0f));
  const float alpha = derive_d_filter_alpha(0.002f, 50.0f);
  CHECK(alpha > 0.0f && alpha < 1.0f);

  // Pole-placement: kp = ωn²/K, kd = 2ζωn/K.
  float kp = 0.0f;
  float kd = 0.0f;
  derive_pd_gains(0.7f, 10.0f, 1.0f, &kp, &kd);
  CHECK(near(kp, 100.0f, 1e-2f));
  CHECK(near(kd, 14.0f, 1e-2f));

  CHECK(derive_obstacle_window(200.0f, 20.0f) == 10);
  CHECK(derive_obstacle_window(1.0f, 20.0f) == 1);
  CHECK(near(derive_decel_per_tick(0.02f), 0.02f));
  CHECK(near(derive_decel_per_tick(-0.5f), 0.0f));

  // Trim sprint: Run sạch → tăng; có mất vạch → giảm, sàn 1.0.
  CHECK(trim_sprint_scale(1.0f, 0) > 1.0f);
  CHECK(trim_sprint_scale(1.2f, 3) < 1.2f);
  CHECK(near(trim_sprint_scale(1.0f, 100), 1.0f));
}

}

int main() {
  test_line_error();
  test_steer();
  test_evasion();
  test_mix();
  test_step_run1();
  test_step_finish();
  test_bake();
  test_normalize_ir();
  test_normalize_tof();
  test_tuning_derive();

  if (failures() == 0) {
    (void)std::printf("test_kernel: ALL PASS\n");
    return 0;
  }
  (void)std::printf("test_kernel: %d FAILED\n", failures());
  return 1;
}
