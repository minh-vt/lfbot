#include <cmath>
#include <cstdio>
#include <vector>

#include "check.hpp"
#include "config.hpp"
#include "course.hpp"
#include "plant.hpp"
#include "suites.hpp"

namespace {

constexpr float kCalmTurn = 0.05F;
constexpr float kHardTurn = 0.6F;

// The middle of stretch `index`, in counts.
float at(float index) {
  return (index + 0.5F) * lf::kStretchCounts;
}

// Run 1 over `length` stretches: calm, except hard stretches and one corner.
lf::Course recorded(int length, const std::vector<int>& hard, int corner) {
  lf::Course course;
  course.start();
  for (int i = 0; i < length; ++i) {
    bool is_hard = false;
    for (const int h : hard) {
      is_hard = is_hard || h == i;
    }
    const lf::Guide guide = i == corner ? lf::Guide::kSearch : lf::Guide::kLine;
    course.note(at(static_cast<float>(i)), is_hard ? kHardTurn : kCalmTurn, guide);
  }
  course.finish();
  course.start();  // now run 2
  return course;
}

void test_run_1_writes_down_effort_and_landmarks() {
  const lf::Course course = recorded(40, {12}, 30);
  const lf::CourseRecord& record = course.record();
  require(course.complete() && course.replaying(), "a finished run 1 makes the next race run 2");
  require(record.length == 40, "one stretch per kStretchMm of line");
  require(record.stretches[12].effort > record.stretches[11].effort * 5,
          "a stretch steered hard is written down as hard");
  require((record.stretches[30].marks & lf::kMarkCorner) != 0 && record.stretches[29].marks == 0,
          "a corner is marked where the search began");
}

void test_an_unfinished_run_1_is_not_a_course() {
  lf::Course course;
  course.start();
  course.note(at(0.0F), kCalmTurn, lf::Guide::kLine);
  course.note(at(5.0F), kCalmTurn, lf::Guide::kLine);
  require(!course.complete(), "a run 1 that did not finish is not kept");
  course.start();
  require(!course.replaying(), "so the next race is run 1 again");

  lf::Course empty;
  require(!empty.load(lf::CourseRecord{}), "an empty record is refused");
}

void test_run_2_goes_faster_only_where_run_1_was_calm() {
  const lf::Course course = recorded(100, {60}, -1);
  require(course.pace(at(0.0F), 0.0F) == lf::kFastPace, "calm ahead: faster than run 1");
  require(course.pace(at(59.0F), 0.0F) == lf::kFastPace,
          "a hard stretch beyond the look-ahead does not slow it yet");
  // Two stretches of look-ahead in kLookaheadTicks.
  const float fast = 2.0F * lf::kStretchCounts / lf::kLookaheadTicks;
  require(course.pace(at(59.0F), fast) == 1.0F,
          "going faster, it sees the hard stretch sooner and drops to run 1's pace");
  require(course.pace(at(60.0F), 0.0F) == 1.0F, "and stays there over it");
  require(course.pace(at(99.0F), fast) == 1.0F,
          "the end of the record is the finish: run 1's pace");

  lf::Course run_1;
  run_1.start();
  require(run_1.pace(at(0.0F), fast) == 1.0F, "run 1 is never sped up");
}

void test_a_landmark_corrects_the_distance() {
  lf::Course course = recorded(100, {70}, 50);
  // Run 2 meets the corner four stretches later by its encoders than run 1 did.
  course.note(at(53.0F), kCalmTurn, lf::Guide::kLine);
  course.note(at(54.0F), kHardTurn, lf::Guide::kSearch);
  course.note(at(54.0F), kCalmTurn, lf::Guide::kLine);
  require(course.pace(at(73.0F), 0.0F) == lf::kFastPace, "four stretches before the hard one");
  require(course.pace(at(74.0F), 0.0F) == 1.0F,
          "the hard stretch is where run 1 met it, counted from the corner");
}

void test_a_detour_or_a_strange_landmark_leaves_it_unsure() {
  lf::Course course = recorded(100, {}, 50);
  course.note(at(20.0F), kCalmTurn, lf::Guide::kVirtual);
  course.note(at(30.0F), kCalmTurn, lf::Guide::kLine);
  require(course.pace(at(31.0F), 0.0F) == 1.0F,
          "after a brick it does not know how far the detour put it off");
  course.note(at(52.0F), kHardTurn, lf::Guide::kSearch);
  course.note(at(52.0F), kCalmTurn, lf::Guide::kLine);
  require(course.pace(at(53.0F), 0.0F) == lf::kFastPace, "the next corner puts it right");

  lf::Course strange = recorded(100, {}, 50);
  strange.note(at(10.0F), kHardTurn, lf::Guide::kSearch);
  strange.note(at(10.0F), kCalmTurn, lf::Guide::kLine);
  require(strange.pace(at(11.0F), 0.0F) == 1.0F,
          "a corner run 1 never met near here: run 1's pace until the next match");
}

// A long straight, a left corner, another long straight: where run 2 can win.
std::vector<Segment> two_straights() {
  return {{0.0F, 0.0F, 2500.0F, 0.0F}, {2500.0F, 0.0F, 2500.0F, 2000.0F}};
}

// Races from the start of the line until it finishes or stops; seconds taken. A brick
// sits on the first straight if brick_x > 0.
float race_from_start(Bench& bench, double brick_x = 0.0) {
  bench.plant = Plant{};
  bench.plant.course = two_straights();
  bench.plant.has_brick = brick_x > 0.0;
  bench.plant.brick_front_x = brick_x;
  require(bench.start_race(), "calibrated and racing");
  int ticks = 0;
  while (bench.phase() == lf::Phase::kRacing && ticks < 120 * static_cast<int>(lf::kControlHz)) {
    bench.tick();
    ++ticks;
  }
  return static_cast<float>(ticks) / static_cast<float>(lf::kControlHz);
}

void test_run_2_is_faster_on_the_same_line() {
  Bench bench;
  const float first = race_from_start(bench);
  require(bench.robot.snapshot().reason == lf::StopReason::kFinish, "run 1 finishes");
  require(bench.robot.snapshot().run == 1, "and it was run 1");
  bench.press();  // acknowledge the finish
  require(bench.outputs.led == lf::Colour::kMagenta, "magenta: ready for run 2");

  const float second = race_from_start(bench);
  const lf::Snapshot snapshot = bench.robot.snapshot();
  std::printf("  run 1 %.2f s, run 2 %.2f s, %u stretches, end x=%.0f y=%.0f\n",
              static_cast<double>(first), static_cast<double>(second),
              static_cast<unsigned>(snapshot.course_stretches), bench.plant.x_mm, bench.plant.y_mm);
  require(snapshot.reason == lf::StopReason::kFinish && snapshot.run == 2, "run 2 finishes");
  // As in the pilot's finish test: it stops a little past the end, never short of it.
  require(bench.plant.y_mm > 1980.0 && bench.plant.y_mm < 2200.0 &&
            std::abs(bench.plant.x_mm - 2500.0) < 60.0,
          "and stops where run 1 did, just past the end of the line");
  require(second < 0.95F * first, "and is faster than run 1");
}

// Run 2 arrives at a brick faster than run 1 ever went; the brick is somewhere run 1
// found a plain line.
void test_run_2_still_rounds_a_brick() {
  Bench bench;
  static_cast<void>(race_from_start(bench));
  bench.press();
  static_cast<void>(race_from_start(bench, 1600.0));
  const lf::Snapshot snapshot = bench.robot.snapshot();
  std::printf("  run 2 with a brick: reason=%d hit=%d\n", static_cast<int>(snapshot.reason),
              static_cast<int>(bench.plant.brick_hit));
  require(snapshot.reason == lf::StopReason::kFinish && snapshot.run == 2,
          "run 2 goes round a new brick and finishes");
  require(!bench.plant.brick_hit, "without touching it");
}

}  // namespace

void run_course() {
  test_run_1_writes_down_effort_and_landmarks();
  test_an_unfinished_run_1_is_not_a_course();
  test_run_2_goes_faster_only_where_run_1_was_calm();
  test_a_landmark_corrects_the_distance();
  test_a_detour_or_a_strange_landmark_leaves_it_unsure();
  test_run_2_is_faster_on_the_same_line();
  test_run_2_still_rounds_a_brick();
  std::puts("course: PASS (record, read-ahead, landmarks, run 2 faster in the plant)");
}
