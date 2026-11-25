#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "config.hpp"
#include "types.hpp"

namespace lf {

// What run 1 met in one kStretchMm of line (ADR-024). It records how hard the robot
// had to steer, not the shape of the line: a stretch is hard because run 1 found it
// hard, whatever the reason.
struct Stretch {
  std::uint8_t effort = 0;  // largest |turn| in the stretch, 255 = kTurnDutyMax
  std::uint8_t marks = 0;   // kMark* bits
};

// The line was lost and found again by searching: a corner.
inline constexpr std::uint8_t kMarkCorner = 1U << 0U;
// Most of the bar went dark: a crossing, a node or a thick mark.
inline constexpr std::uint8_t kMarkDark = 1U << 1U;
// Run 1 drove round a brick here. Bricks move between runs, so this is neither a
// landmark nor a known line.
inline constexpr std::uint8_t kMarkBrick = 1U << 2U;

// Run 1 as kept between runs, and in flash between power cycles. Plain data: the
// platform stores and loads it, the core gives it meaning.
struct CourseRecord {
  std::uint32_t length = 0;  // stretches in use
  std::array<Stretch, kCourseStretches> stretches{};
};

// Run 1 writes down where it had to work; run 2 reads ahead in that record and goes
// faster where nothing hard is coming. Run 2 knows where it is only by distance, and
// corrects that distance each time it meets a corner or dark patch in the order run 1
// met them. Until it is sure again, it runs at run 1's pace.
class Course {
public:
  // At the start of a race: run 2 if a finished run 1 is held, otherwise a new run 1.
  void start();
  // Run 1 reached the finish: what it wrote down becomes the course.
  void finish();
  [[nodiscard]] bool load(const CourseRecord& record);
  [[nodiscard]] const CourseRecord& record() const { return record_; }
  [[nodiscard]] bool complete() const { return complete_; }
  [[nodiscard]] bool replaying() const { return replaying_; }

  // Every racing tick, after steering: run 1 records, run 2 finds its place.
  void note(float s_counts, float turn, Guide guide);
  // How much faster than run 1 to follow a plain line now: 1 in run 1, and in run 2
  // whenever something hard, unknown or unsure lies within kLookaheadTicks.
  [[nodiscard]] float pace(float s_counts, float counts_per_tick) const;

private:
  void write_down(float s_counts, float turn, Guide guide);
  void find_place(float s_counts, Guide guide);
  [[nodiscard]] bool calm(std::size_t first, std::size_t last) const;

  CourseRecord record_;
  bool complete_ = false;
  bool replaying_ = false;
  Guide previous_ = Guide::kLine;
  // Run 2 only: where the record says the robot is, minus where the encoders say.
  float shift_counts_ = 0.0F;
  bool sure_ = true;
  std::size_t next_landmark_ = 0;  // landmarks before this one are behind the robot
};

}  // namespace lf
