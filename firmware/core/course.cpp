#include "course.hpp"

#include <algorithm>
#include <cmath>

namespace lf {
namespace {

constexpr float kEffortFull = 255.0F;

std::uint8_t effort_of(float turn) {
  const float fraction = std::clamp(std::abs(turn) / kTurnDutyMax, 0.0F, 1.0F);
  return static_cast<std::uint8_t>(std::lround(fraction * kEffortFull));
}

std::size_t stretch_at(float s_counts) {
  return static_cast<std::size_t>(std::max(0.0F, s_counts) / kStretchCounts);
}

// The mark a guide leaves where it begins, if it is a landmark.
std::uint8_t landmark_of(Guide guide) {
  switch (guide) {
    case Guide::kSearch:
      return kMarkCorner;
    case Guide::kMemory:
      return kMarkDark;
    case Guide::kLine:
    case Guide::kStraight:
    case Guide::kVirtual:
      break;
  }
  return 0;
}

}  // namespace

void Course::start() {
  replaying_ = complete_;
  if (!replaying_) {
    record_ = {};
  }
  previous_ = Guide::kLine;
  shift_counts_ = 0.0F;
  sure_ = true;
  next_landmark_ = 0;
}

void Course::finish() {
  if (!replaying_) {
    complete_ = record_.length > 0;
  }
}

bool Course::load(const CourseRecord& record) {
  if (record.length == 0 || record.length > kCourseStretches) {
    return false;
  }
  record_ = record;
  complete_ = true;
  return true;
}

void Course::note(float s_counts, float turn, Guide guide) {
  if (replaying_) {
    find_place(s_counts, guide);
  } else {
    write_down(s_counts, turn, guide);
  }
  previous_ = guide;
}

void Course::write_down(float s_counts, float turn, Guide guide) {
  const std::size_t index = stretch_at(s_counts);
  if (index >= kCourseStretches) {
    return;  // longer than the record holds: run 2 runs the rest at run 1's pace
  }
  // Stretches skipped over are already empty from start().
  record_.length = std::max(record_.length, static_cast<std::uint32_t>(index + 1));
  Stretch& stretch = record_.stretches[index];
  stretch.effort = std::max(stretch.effort, effort_of(turn));
  if (guide != previous_) {
    stretch.marks |= landmark_of(guide);
  }
  if (guide == Guide::kVirtual) {
    stretch.marks |= kMarkBrick;
  }
}

// A landmark is matched to the nearest one of its kind that run 1 met after the last
// match, if that one is close to where the record expects the robot. A detour, or a
// landmark run 1 never met there, leaves the robot unsure of where it is.
void Course::find_place(float s_counts, Guide guide) {
  if (guide == previous_) {
    return;
  }
  if (guide == Guide::kVirtual) {
    sure_ = false;
    return;
  }
  const std::uint8_t mark = landmark_of(guide);
  if (mark == 0) {
    return;
  }
  const float here = s_counts + shift_counts_;
  std::size_t best = record_.length;
  float best_gap = kLandmarkWindowCounts;
  for (std::size_t i = next_landmark_; i < record_.length; ++i) {
    const float there = (static_cast<float>(i) + 0.5F) * kStretchCounts;
    if (there - here > kLandmarkWindowCounts) {
      break;
    }
    const float gap = std::abs(there - here);
    if ((record_.stretches[i].marks & mark) != 0 && gap <= best_gap) {
      best = i;
      best_gap = gap;
    }
  }
  sure_ = best < record_.length;
  if (sure_) {
    shift_counts_ = (static_cast<float>(best) + 0.5F) * kStretchCounts - s_counts;
    next_landmark_ = best + 1;
  }
}

float Course::pace(float s_counts, float counts_per_tick) const {
  if (!replaying_ || !sure_) {
    return 1.0F;
  }
  const float here = s_counts + shift_counts_;
  const std::size_t first = stretch_at(here);
  const std::size_t last = stretch_at(here + kLookaheadTicks * std::max(0.0F, counts_per_tick));
  // Past the end of the record lies the finish, or line run 1 never wrote down.
  if (last >= record_.length) {
    return 1.0F;
  }
  return calm(first, last) ? kFastPace : 1.0F;
}

bool Course::calm(std::size_t first, std::size_t last) const {
  const auto hard = static_cast<std::uint8_t>(std::lround(kHardEffort * kEffortFull));
  for (std::size_t i = first; i <= last; ++i) {
    const Stretch& stretch = record_.stretches[i];
    if (stretch.marks != 0 || stretch.effort > hard) {
      return false;
    }
  }
  return true;
}

}  // namespace lf
