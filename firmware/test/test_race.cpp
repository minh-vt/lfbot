#include <array>
#include <cmath>
#include <cstdio>

#include "check.hpp"
#include "config.hpp"
#include "line.hpp"
#include "plant.hpp"
#include "suites.hpp"

namespace {

// The robot sits at the start of the line and one press sets it going. Runs until
// it finishes, stops, or the time is up.
struct Outcome {
  lf::Phase phase = lf::Phase::kIdle;
  lf::StopReason reason = lf::StopReason::kNone;
  float x_mm = 0.0F;
  float y_mm = 0.0F;
  float seconds = 0.0F;
  lf::RaceState race = lf::RaceState::kFailed;
};

void press(Bench& bench) {
  for (int tick = 0; tick < 40; ++tick) {
    bench.tick(true);
  }
  for (int tick = 0; tick < 40; ++tick) {
    bench.tick(false);
  }
}

// Runs until the robot reaches `phase` or stops; false if time ran out first.
bool run_until(Bench& bench, lf::Phase phase, float limit_s) {
  const auto ticks = static_cast<int>(limit_s * static_cast<float>(lf::kControlHz));
  for (int tick = 0; tick < ticks; ++tick) {
    bench.tick();
    const lf::Phase now = bench.robot.snapshot().phase;
    if (now == phase || now == lf::Phase::kStopped || now == lf::Phase::kFinished) {
      return now == phase;
    }
  }
  return false;
}

// One press calibrates and the robot waits; a second press sets it going. Runs until
// it finishes, stops, or the time is up.
Outcome race(Bench& bench, float limit_s) {
  bench.run_ms(100);
  press(bench);
  static_cast<void>(run_until(bench, lf::Phase::kReady, 60.0F));
  if (bench.robot.snapshot().phase == lf::Phase::kReady) {
    press(bench);
  }
  Outcome outcome;
  const auto ticks = static_cast<int>(limit_s * static_cast<float>(lf::kControlHz));
  for (int tick = 0; tick < ticks; ++tick) {
    bench.tick();
    const lf::Phase phase = bench.robot.snapshot().phase;
    if (phase == lf::Phase::kFinished || phase == lf::Phase::kStopped) {
      break;
    }
    outcome.seconds = static_cast<float>(tick) / static_cast<float>(lf::kControlHz);
  }
  const lf::Snapshot snapshot = bench.robot.snapshot();
  outcome.phase = snapshot.phase;
  outcome.reason = snapshot.reason;
  outcome.race = snapshot.race;
  outcome.x_mm = static_cast<float>(bench.plant.x_mm);
  outcome.y_mm = static_cast<float>(bench.plant.y_mm);
  return outcome;
}

std::vector<Segment> straight(float length) {
  return {{0.0F, 0.0F, length, 0.0F}};
}

std::vector<Segment> left_corner(float first, float second) {
  return {{0.0F, 0.0F, first, 0.0F}, {first, 0.0F, first, second}};
}

std::vector<Segment> right_corner(float first, float second) {
  return {{0.0F, 0.0F, first, 0.0F}, {first, 0.0F, first, -second}};
}

// A line running across ours at x = 400, long enough to reach both edges of the bar.
std::vector<Segment> crossing() {
  return {{0.0F, 0.0F, 900.0F, 0.0F}, {400.0F, -150.0F, 400.0F, 150.0F}};
}

void test_it_finds_the_line_follows_it_and_stops_at_the_end() {
  struct Case {
    const char* name = "";
    Surface surface{};
  };
  const std::array<Case, 4> cases = {
    Case{"a light floor and a dark line", Surface::uniform(3300.0F, 700.0F)},
    Case{"a dark floor and a bright line", Surface::uniform(700.0F, 3300.0F)},
    Case{"a sunlit floor with a pale line", Surface::uniform(3900.0F, 3000.0F)},
    Case{"a dim room", Surface::uniform(1500.0F, 600.0F)},
  };
  for (const Case& test : cases) {
    Bench bench;
    bench.plant.course = straight(800.0F);
    bench.plant.surface = test.surface;
    const Outcome outcome = race(bench, 90.0F);
    std::printf("  [%s] phase=%d reason=%d x=%.0f y=%.0f t=%.1fs race=%d\n", test.name,
                static_cast<int>(outcome.phase), static_cast<int>(outcome.reason), outcome.x_mm,
                outcome.y_mm, outcome.seconds, static_cast<int>(outcome.race));
    require(outcome.phase == lf::Phase::kFinished && outcome.reason == lf::StopReason::kFinish,
            test.name);
    require(outcome.x_mm > 780.0F && outcome.x_mm < 1000.0F && std::abs(outcome.y_mm) < 40.0F,
            "and it stops at the end of the line, not before it");
  }
}

void test_it_starts_from_a_poor_placement() {
  Bench bench;
  bench.plant.course = straight(800.0F);
  bench.plant.y_mm = 9.0;
  bench.plant.yaw_rad = 0.06;
  const Outcome outcome = race(bench, 90.0F);
  require(outcome.reason == lf::StopReason::kFinish,
          "a few millimetres and degrees off the line is fine");
}

void test_it_turns_left_and_right_corners() {
  for (const bool left : {true, false}) {
    Bench bench;
    bench.plant.course = left ? left_corner(400.0F, 500.0F) : right_corner(400.0F, 500.0F);
    const Outcome outcome = race(bench, 120.0F);
    std::printf("  [%s corner] phase=%d reason=%d x=%.0f y=%.0f t=%.1fs\n", left ? "left" : "right",
                static_cast<int>(outcome.phase), static_cast<int>(outcome.reason), outcome.x_mm,
                outcome.y_mm, outcome.seconds);
    require(outcome.reason == lf::StopReason::kFinish,
            "a 90 degree corner is turned and the race finishes");
    require(std::abs(outcome.y_mm) > 400.0F, "and it is by the end of the second leg");
  }
}

void test_it_crosses_a_crossing_line_without_turning_onto_it() {
  Bench bench;
  bench.plant.course = crossing();
  const Outcome outcome = race(bench, 120.0F);
  std::printf("  [crossing] phase=%d reason=%d x=%.0f y=%.0f t=%.1fs\n",
              static_cast<int>(outcome.phase), static_cast<int>(outcome.reason), outcome.x_mm,
              outcome.y_mm, outcome.seconds);
  require(outcome.reason == lf::StopReason::kFinish && outcome.x_mm > 880.0F &&
            std::abs(outcome.y_mm) < 40.0F,
          "a line across ours is crossed and the race goes on to the real end");
}

// A line across ours with nothing straight on: the robot must not take it for the
// end of the line, and must turn onto it.
void test_a_t_junction_is_turned_not_taken_for_the_end() {
  Bench bench;
  bench.plant.course = {{0.0F, 0.0F, 400.0F, 0.0F}, {400.0F, -300.0F, 400.0F, 300.0F}};
  const Outcome outcome = race(bench, 120.0F);
  std::printf("  [t junction] phase=%d reason=%d x=%.0f y=%.0f t=%.1fs\n",
              static_cast<int>(outcome.phase), static_cast<int>(outcome.reason), outcome.x_mm,
              outcome.y_mm, outcome.seconds);
  require(outcome.reason == lf::StopReason::kFinish,
          "a T junction is turned and the race ends at an arm");
  require(outcome.y_mm < -250.0F, "and it turns right when both arms are open");
}

// The outermost eyes sit near the ADC rail and flicker; the corner must still be taken.
void test_a_corner_is_taken_with_noisy_outer_eyes() {
  for (const bool left : {true, false}) {
    Bench bench;
    bench.plant.course = left ? left_corner(400.0F, 500.0F) : right_corner(400.0F, 500.0F);
    bench.plant.saturated[0] = true;
    bench.plant.saturated[7] = true;
    const Outcome outcome = race(bench, 120.0F);
    require(outcome.reason == lf::StopReason::kFinish && std::abs(outcome.y_mm) > 400.0F,
            "a corner is turned even with the two outer eyes lost");
  }
}

void test_a_single_flickering_eye_is_not_a_second_line() {
  std::array<float, lf::kIrCount> dark{};
  dark[3] = 1.0F;
  dark[4] = 1.0F;
  dark[7] = 1.0F;  // an eye near the rail crossing its threshold
  const lf::LineView view = lf::read_line(dark, 0.0F);
  require(view.shape == lf::LineShape::kLine, "a lone eye beside a real patch is ignored");
  require(std::abs(view.position) < 0.1F, "and does not pull the position");
}

void test_blind_eyes_are_tolerated_up_to_a_point() {
  Bench two;
  two.plant.course = straight(800.0F);
  two.plant.saturated[0] = true;
  two.plant.saturated[7] = true;
  require(race(two, 90.0F).reason == lf::StopReason::kFinish, "two saturated eyes: still races");

  Bench four;
  four.plant.course = straight(800.0F);
  for (const std::size_t eye : {0U, 1U, 2U, 7U}) {
    four.plant.saturated[eye] = true;
  }
  const Outcome refused = race(four, 90.0F);
  require(refused.phase == lf::Phase::kStopped && refused.reason == lf::StopReason::kScanFailed,
          "four saturated eyes: refuses, says why, and does not drive off");
  require(std::abs(refused.x_mm) < 30.0F, "having only turned in place");
}

void test_no_line_under_the_robot_is_refused() {
  Bench bench;
  bench.plant.course = {{0.0F, 500.0F, 800.0F, 500.0F}};  // a line, but 50 cm away
  const Outcome outcome = race(bench, 60.0F);
  require(outcome.reason == lf::StopReason::kScanFailed, "no line to learn from is a scan failure");
}

void test_it_calibrates_then_waits_for_a_second_press() {
  Bench bench;
  bench.plant.course = straight(800.0F);
  bench.run_ms(100);
  press(bench);
  require(run_until(bench, lf::Phase::kReady, 60.0F), "one press calibrates and reaches ready");
  require(bench.outputs.led.colour == lf::Colour::kPurple, "ready shows purple");
  const double x_before = bench.plant.x_mm;
  const std::size_t log_before = bench.robot.log().size();
  bench.run_ms(30000);
  require(bench.robot.snapshot().phase == lf::Phase::kReady, "and it stays ready, however long");
  require(bench.outputs.motor.mode != lf::MotorMode::kDrive &&
            std::abs(bench.plant.x_mm - x_before) < 1.0,
          "standing still");
  require(bench.robot.log().size() == log_before, "waiting does not fill the log");
  press(bench);
  require(bench.robot.snapshot().phase == lf::Phase::kRunning, "the second press sets it going");
  require(bench.outputs.led.colour == lf::Colour::kGreen, "and shows green");
  bench.run_ms(60000);
  require(bench.robot.snapshot().reason == lf::StopReason::kFinish, "and it finishes the line");
}

// A filled square node at a 90 degree corner: the bar goes fully dark, then the line
// turns. The robot must turn at the node, not overshoot it.
void test_a_filled_node_corner_is_turned() {
  Bench bench;
  bench.plant.course = {{0.0F, 0.0F, 300.0F, 0.0F}, {300.0F, 30.0F, 300.0F, 600.0F}};
  bench.plant.dark_squares.push_back({300.0F, 0.0F, 25.0F, 0.0F});
  const Outcome outcome = race(bench, 120.0F);
  std::printf("  [filled node] phase=%d reason=%d x=%.0f y=%.0f t=%.1fs\n",
              static_cast<int>(outcome.phase), static_cast<int>(outcome.reason), outcome.x_mm,
              outcome.y_mm, outcome.seconds);
  require(outcome.reason == lf::StopReason::kFinish, "a filled node is turned and the race ends");
  require(outcome.y_mm > 300.0F, "onto the leg the line continues on");
}

// A brick on the line is bypassed on the right and the line re-acquired.
void test_a_brick_is_bypassed_on_the_right() {
  Bench bench;
  bench.plant.course = {{0.0F, 0.0F, 1300.0F, 0.0F}};
  bench.plant.brick_front_x = 550.0F;
  bench.plant.has_brick = true;
  const Outcome outcome = race(bench, 150.0F);
  std::printf("  [brick] phase=%d reason=%d x=%.0f y=%.0f t=%.1fs\n",
              static_cast<int>(outcome.phase), static_cast<int>(outcome.reason), outcome.x_mm,
              outcome.y_mm, outcome.seconds);
  require(outcome.reason == lf::StopReason::kFinish, "a brick is bypassed and the race goes on");
  require(outcome.x_mm > 1200.0F, "past the brick to the end of the line");
}

// The shape of the follow law: full speed and no turn on a centred line, steering
// and slowing as the line moves or drifts towards an edge.
void test_follow_law_steers_and_slows() {
  require(lf::follow_turn(0.0F, 0.0F) == 0.0F, "a centred line needs no turn");
  require(lf::follow_turn(3.5F, 0.0F) < 0.0F, "a line to the right turns right");
  require(lf::follow_turn(0.0F, 10.0F) < 0.0F, "a drift to the right also turns right");
  require(lf::follow_turn(-3.5F, 0.0F) > 0.0F, "and a line to the left turns left");
  require(std::abs(lf::follow_turn(0.0F, 100.0F)) <= lf::kTurnDutyMax + 1e-4F,
          "the turn duty is clamped");
  require(lf::follow_forward(0.0F, 0.0F) == lf::kCruiseDuty, "full duty on a straight line");
  require(lf::follow_forward(3.5F, 0.0F) < lf::follow_forward(0.0F, 0.0F),
          "an offset line slows down");
  require(lf::follow_forward(0.0F, 10.0F) < lf::follow_forward(0.0F, 0.0F),
          "a drifting line slows down");
  require(lf::follow_forward(100.0F, 100.0F) >= lf::kMinFollowFraction * lf::kCruiseDuty - 1.0F,
          "but never below the follow floor");
}

void test_the_button_stops_a_calibration_and_a_race() {
  Bench scanning;
  scanning.plant.course = straight(800.0F);
  scanning.run_ms(100);
  press(scanning);
  scanning.run_ms(800);
  require(scanning.robot.snapshot().phase == lf::Phase::kRunning, "scanning right after the press");
  for (int tick = 0; tick < 40; ++tick) {
    scanning.tick(true);
  }
  require(scanning.robot.snapshot().reason == lf::StopReason::kButton,
          "a press stops a calibration");

  Bench racing;
  racing.plant.course = straight(800.0F);
  racing.run_ms(100);
  press(racing);
  require(run_until(racing, lf::Phase::kReady, 60.0F), "calibrated");
  press(racing);
  racing.run_ms(4000);
  require(racing.robot.snapshot().phase == lf::Phase::kRunning, "racing");
  for (int tick = 0; tick < 40 && racing.robot.snapshot().phase == lf::Phase::kRunning; ++tick) {
    racing.tick(true);
  }
  require(racing.robot.snapshot().reason == lf::StopReason::kButton, "a press stops a race");
  require(racing.outputs.motor.mode == lf::MotorMode::kBrake, "and brakes");
}

}  // namespace

void run_race() {
  test_it_finds_the_line_follows_it_and_stops_at_the_end();
  test_it_starts_from_a_poor_placement();
  test_it_turns_left_and_right_corners();
  test_it_crosses_a_crossing_line_without_turning_onto_it();
  test_a_t_junction_is_turned_not_taken_for_the_end();
  test_a_corner_is_taken_with_noisy_outer_eyes();
  test_a_single_flickering_eye_is_not_a_second_line();
  test_blind_eyes_are_tolerated_up_to_a_point();
  test_no_line_under_the_robot_is_refused();
  test_it_calibrates_then_waits_for_a_second_press();
  test_a_filled_node_corner_is_turned();
  test_a_brick_is_bypassed_on_the_right();
  test_follow_law_steers_and_slows();
  test_the_button_stops_a_calibration_and_a_race();
  std::puts("race: PASS (scan, follow, corners, crossing, finish, blind eyes)");
}
