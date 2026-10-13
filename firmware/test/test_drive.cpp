#include <cmath>
#include <cstdio>

#include "drive/plan.hpp"
#include "drive/step.hpp"
#include "sensing/layout.hpp"
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
  CHECK(steer_to_line(0.2f, 0.0f) > 0.0f);   // lệch phải → lái phải
  CHECK(steer_to_line(-0.2f, 0.0f) < 0.0f);  // lệch trái → lái trái
  // Damping: heading cùng dấu error → lái mạnh hơn.
  CHECK(steer_to_line(0.01f, 0.005f) > steer_to_line(0.01f, 0.0f));
  // Bão hòa ở ±1.
  CHECK(near(steer_to_line(10.0f, 0.0f), 1.0f));
  CHECK(near(steer_to_line(-10.0f, 0.0f), -1.0f));
}

void test_avoid() {
  using namespace lf::drive;
  Readings frame;
  frame.line_certainty = 1.0f;
  frame.dt = 0.01f;
  frame.obstacle_danger = 1.0f;  // vật cản sát
  frame.obstacle_side = 0.8f;    // bên phải → lái tránh sang trái
  const DriveState state;
  const RacePlan plan;
  const Tuning cfg;
  const auto result = step(state, frame, plan, cfg);
  CHECK(!result.command.brake);
  CHECK(result.command.left_duty > result.command.right_duty);  // né sang trái
}

void test_field_continuous() {
  using namespace lf::drive;
  const RacePlan plan;
  // Cùng side, quét danger: steer liên tục (không nhảy bậc) và đổi hướng theo bên.
  float max_jump = 0.0f;
  float prev = 0.0f;
  for (int i = 0; i <= 100; ++i) {
    Readings frame;
    frame.obstacle_danger = 0.01f * static_cast<float>(i);
    frame.obstacle_side = 0.7f;  // vật cản bên phải
    DriveState s;
    const FieldOut out = apply_field(0.0f, frame, s);
    if (i > 0) {
      max_jump = std::max(max_jump, std::abs(out.steer - prev));
    }
    prev = out.steer;
  }
  CHECK(max_jump < 0.02f);  // liên tục: bước danger 0.01 → steer đổi rất nhỏ
  CHECK(prev < 0.0f);       // danger lớn → lái tránh sang trái (steer âm)
  // Không có vật cản → giữ nguyên lái bám line, không giảm tốc.
  const Readings clear;
  DriveState s2;
  const FieldOut passthrough = apply_field(0.42f, clear, s2);
  CHECK(near(passthrough.steer, 0.42f));
  CHECK(near(passthrough.speed_scale, 1.0f));
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
  record(plan, 0.0f, 0.0f, 1.0f, 0.0f);
  record(plan, 1.0f, 0.0f, 1.0f, 0.0f);
  // thẳng nhanh (speed 1.0) rồi cua chậm (speed 0.2) -> backward-pass giảm ga trước cua.
  record(plan, 2.0f, 0.0f, 0.2f, 0.0f);
  record(plan, 3.0f, 0.0f, 0.2f, 0.0f);
  record(plan, 4.0f, 0.0f, 0.2f, 0.0f);
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

void test_estimate_motion() {
  using namespace lf::drive;
  Readings frame;
  frame.enc_l = 10;
  frame.enc_r = 10;
  const DriveState state;
  const Tuning cfg;
  const MotionEstimate motion = estimate_motion(frame, state, cfg, 0.01f);
  CHECK(near(motion.tick, 10.0f));
  CHECK(motion.speed > 0.0f);  // có encoder increment → tốc độ dương
  // Hội tụ về v_raw = 10 tick / 0.01s = 1000 tick/s.
  DriveState s;
  float speed = 0.0f;
  for (int i = 0; i < 4000; ++i) {
    s.speed = speed;
    speed = estimate_motion(frame, s, cfg, 0.01f).speed;
  }
  CHECK(near(speed, 1000.0f, 2.0f));
}

void test_estimate_line() {
  using namespace lf::drive;
  Readings frame;
  frame.line_offset = 0.5f;
  frame.line_certainty = 1.0f;
  const RacePlan plan;  // rỗng → lateral_at trả 0
  const Tuning cfg;
  DriveState next;
  float offset = 0.0f;
  for (int i = 0; i < 400; ++i) {
    offset = estimate_line(frame, next, plan, 0.0f, 0.01f, cfg).offset;
  }
  CHECK(near(next.line_estimate, 0.5f, 1e-2f));  // hội tụ về offset sensor
  CHECK(offset > 0.0f);                          // dương = vạch lệch phải
  CHECK(offset < 0.5f);                          // có trọng số tin cậy (< 1)
}

void test_normalize_ir() {
  lf::sensing::IrReadings readings;
  for (auto& value : readings.raw) {
    value = 4095;  // toàn trắng
  }
  auto result = lf::sensing::normalize_ir(readings, lf::sensing::kRobotLayout);
  CHECK(result.line_certainty < 1e-3f);
  readings.raw[7] = 0;  // vạch ở cảm biến phải ngoài
  result = lf::sensing::normalize_ir(readings, lf::sensing::kRobotLayout);
  CHECK(result.line_certainty > 0.0f);
  CHECK(result.line_offset > 0.9f);
}

void test_normalize_tof() {
  lf::drive::Readings frame;
  // obstacle trái rõ (d_left=100 << d_right=300) → side âm, liên tục (không slam −1).
  lf::sensing::normalize_tof(frame, lf::sensing::kRobotLayout, 300.0f, 100.0f, 300.0f);
  CHECK(frame.obstacle_danger > 0.0f);
  CHECK(near(frame.obstacle_side, -0.5f, 1e-2f));
  // đối xứng → side = 0 (không biết né bên nào → đi thẳng, danger vẫn giảm ga).
  lf::sensing::normalize_tof(frame, lf::sensing::kRobotLayout, 100.0f, 100.0f, 100.0f);
  CHECK(near(frame.obstacle_side, 0.0f, 1e-2f));
  // ngoài tầm → danger 0, side 0.
  const float far = lf::sensing::kRobotLayout.tof_range_mm + 1.0f;
  lf::sensing::normalize_tof(frame, lf::sensing::kRobotLayout, far, far, far);
  CHECK(near(frame.obstacle_danger, 0.0f) && near(frame.obstacle_side, 0.0f));
}

void test_geometry_matches_layout() {
  // Default RobotTuning phải khớp hằng suy ra từ layout SSoT (chống drift).
  const lf::drive::RobotTuning robot;
  CHECK(near(robot.geometry_l, lf::sensing::geometry_l(lf::sensing::kRobotLayout)));
  CHECK(near(robot.geometry_s, lf::sensing::geometry_s(lf::sensing::kRobotLayout)));
}

}

int main() {
  test_line_offset();
  test_steer();
  test_avoid();
  test_field_continuous();
  test_mix();
  test_step_run1();
  test_step_finish();
  test_plan_speed();
  test_estimate_motion();
  test_estimate_line();
  test_normalize_ir();
  test_normalize_tof();
  test_geometry_matches_layout();

  if (failures() == 0) {
    (void)std::printf("test_drive: ALL PASS\n");
    return 0;
  }
  (void)std::printf("test_drive: %d FAILED\n", failures());
  return 1;
}
