#include <cmath>
#include <cstdio>

#include "drive/plan.hpp"
#include "drive/step.hpp"
#include "sensing/normalize.hpp"

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

void test_line_offset() {
  using namespace lf::drive;
  Readings frame;
  frame.line_certainty = 1.0f;
  frame.line_offset = 0.5f;
  DriveState state;
  for (int i = 0; i < 50; ++i) {
    (void)fuse_line_offset(frame, state);
  }
  CHECK(near(state.line_estimate, 0.5f, 1e-2f));  // tin cậy cao → hội tụ về offset
  frame.line_certainty = 0.0f;
  frame.line_offset = -1.0f;
  (void)fuse_line_offset(frame, state);
  CHECK(near(state.line_estimate, 0.5f, 1e-2f));  // mất vạch → giữ nguyên (không slam ±1)
}

void test_steer() {
  using namespace lf::drive;
  const Tuning cfg;
  DriveState state;
  // heading = 0 → thuần P: lệch phải → lái dương.
  CHECK(steer_to_line(0.2f, 0.0f, 10.0f, state, cfg) > 0.0f);
  // Gain-scheduling: cùng error, v nhỏ hơn → |steer| lớn hơn (kp ∝ 1/v²).
  const float slow = steer_to_line(0.05f, 0.0f, 5.0f, state, cfg);
  const float fast = steer_to_line(0.05f, 0.0f, 20.0f, state, cfg);
  CHECK(std::abs(slow) > std::abs(fast));
  // Damping: heading cùng dấu error → lái mạnh hơn (thêm term ψ).
  CHECK(steer_to_line(0.01f, 0.005f, 10.0f, state, cfg) >
        steer_to_line(0.01f, 0.0f, 10.0f, state, cfg));
}

void test_avoid() {
  using namespace lf::drive;
  Readings frame;
  frame.line_certainty = 1.0f;
  frame.dt = 0.01f;
  frame.obstacle_danger = 1.0f;  // vật cản sát
  frame.obstacle_side = 0.8f;    // bên phải
  const DriveState state;
  const RacePlan plan;
  const Tuning cfg;
  // Tick 1: phát hiện → chuyển kBrake. Tick 2: phanh.
  const auto r1 = step(state, frame, plan, cfg);
  const auto r2 = step(r1.state, frame, plan, cfg);
  CHECK(r2.command.brake);  // thấy vật cản gần → phanh trước (ticket 36)
}

void test_mix() {
  using namespace lf::drive;
  const auto straight = drive_motors(1.0f, 0.0f, false);
  CHECK(near(straight.left_duty, 1.0f) && near(straight.right_duty, 1.0f));
  const auto stopped = drive_motors(0.5f, 0.0f, true);
  CHECK(stopped.brake && near(stopped.left_duty, 0.0f) && near(stopped.right_duty, 0.0f));
}

void test_step_run1() {
  using namespace lf::drive;
  Readings frame;
  frame.line_certainty = 1.0f;
  frame.line_offset = 0.0f;
  frame.dt = 0.01f;
  const DriveState state;
  const RacePlan plan;
  const Tuning cfg;
  const auto result = step(state, frame, plan, cfg);
  CHECK(!result.command.brake);
  CHECK(near(result.command.left_duty, result.command.right_duty, 1e-2f));
  CHECK(result.command.left_duty > 0.0f);
}

void test_step_finish() {
  using namespace lf::drive;
  Readings frame;
  frame.line_certainty = 1.0f;
  frame.finish_detected = true;
  const DriveState state;
  const RacePlan plan;
  const Tuning cfg;
  const auto result = step(state, frame, plan, cfg);
  CHECK(result.state.finish_latched);
  CHECK(result.command.brake);
}

void test_plan_speed() {
  using namespace lf::drive;
  RacePlan plan;
  record(plan, 0.0f, 0.0f, 0.0f);
  record(plan, 1.0f, 0.0f, 0.0f);
  // cua: giữ steer lớn vài tick để line_curve (low-pass) đủ lớn
  record(plan, 2.0f, 1.0f, 0.0f);
  record(plan, 3.0f, 1.0f, 0.0f);
  record(plan, 4.0f, 1.0f, 0.0f);
  Tuning cfg;
  cfg.robot.speed_limit_table = SpeedLimitTable{};
  cfg.robot.speed_limit_table.add(0.0f, 1.0f);
  cfg.robot.speed_limit_table.add(0.5f, 0.4f);
  cfg.track.brake_limit = 0.2f;
  plan_speed(plan, cfg);
  CHECK(plan.length == 5);
  CHECK(plan.ready);
  CHECK(plan.speed_limit[4] < plan.speed_limit[0]);  // cua → giảm ga
  CHECK(plan.speed_limit[3] < plan.speed_limit[1]);  // backward-pass: giảm trước cua
}

void test_normalize_ir() {
  lf::sensing::IrReadings readings;
  for (auto& value : readings.raw) {
    value = 4095;  // toàn trắng
  }
  auto result = lf::sensing::normalize_ir(readings);
  CHECK(result.line_certainty < 1e-3f);
  readings.raw[7] = 0;  // vạch ở cảm biến phải ngoài
  result = lf::sensing::normalize_ir(readings);
  CHECK(result.line_certainty > 0.0f);
  CHECK(result.line_offset > 0.9f);
}

void test_normalize_tof() {
  lf::drive::Readings frame;
  // obstacle trái rõ (d_left=100 << d_right=300) → side âm, liên tục (không slam −1).
  lf::sensing::normalize_tof(frame, 300.0f, 300.0f, 100.0f, 300.0f);
  CHECK(frame.obstacle_danger > 0.0f);
  CHECK(near(frame.obstacle_side, -0.5f, 1e-2f));
  // đối xứng → side = 0 (không biết né bên nào → đi thẳng, danger vẫn giảm ga).
  lf::sensing::normalize_tof(frame, 300.0f, 100.0f, 100.0f, 100.0f);
  CHECK(near(frame.obstacle_side, 0.0f, 1e-2f));
  // ngoài tầm → danger 0, side 0.
  lf::sensing::normalize_tof(frame, 300.0f, 300.0f, 300.0f, 300.0f);
  CHECK(near(frame.obstacle_danger, 0.0f) && near(frame.obstacle_side, 0.0f));
}

}

int main() {
  test_line_offset();
  test_steer();
  test_avoid();
  test_mix();
  test_step_run1();
  test_step_finish();
  test_plan_speed();
  test_normalize_ir();
  test_normalize_tof();

  if (failures() == 0) {
    (void)std::printf("test_drive: ALL PASS\n");
    return 0;
  }
  (void)std::printf("test_drive: %d FAILED\n", failures());
  return 1;
}
