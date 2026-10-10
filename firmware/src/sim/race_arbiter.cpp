#include "race_arbiter.hpp"

#include <algorithm>
#include <cmath>

namespace lf::sim {

void RaceArbiter::reset(const TrackManifest& manifest) {
  manifest_ = manifest;
  elapsed_s_ = 0.0f;
  line_lost_s_ = 0.0f;
  completed_ = false;
  lap_time_s_ = 0.0f;
  infraction_ = Infraction::kNone;
}

void RaceArbiter::observe(float x, float y, float v, bool line_detected, float dt) {
  if (completed_ || infraction_ != Infraction::kNone) {
    return;  // đã đóng băng
  }
  elapsed_s_ += dt;

  if (!line_detected) {
    line_lost_s_ += dt;
    if (line_lost_s_ > kLineLostTimeoutS) {
      infraction_ = Infraction::kLineLostTimeout;
      return;
    }
  } else {
    line_lost_s_ = 0.0f;
  }

  if (elapsed_s_ > kTimeLimitS) {
    infraction_ = Infraction::kTimeLimitExceeded;
    return;
  }

  if (x < kRobotRadiusMm || y < kRobotRadiusMm || x > manifest_.arena_width_mm - kRobotRadiusMm ||
      y > manifest_.arena_height_mm - kRobotRadiusMm) {
    infraction_ = Infraction::kOutOfBounds;
    return;
  }

  for (const auto& obstacle : manifest_.obstacles) {
    const float half_w = obstacle.width_mm * 0.5f;
    const float half_h = obstacle.height_mm * 0.5f;
    const float cx = std::max(obstacle.x_mm - half_w, std::min(x, obstacle.x_mm + half_w));
    const float cy = std::max(obstacle.y_mm - half_h, std::min(y, obstacle.y_mm + half_h));
    const float dx = x - cx;
    const float dy = y - cy;
    if (dx * dx + dy * dy <= kRobotRadiusMm * kRobotRadiusMm) {
      infraction_ = Infraction::kObstacleCollision;
      return;
    }
  }

  const bool in_box = (x >= manifest_.finish_min_x && x <= manifest_.finish_max_x &&
                       y >= manifest_.finish_min_y && y <= manifest_.finish_max_y);
  if (in_box && std::abs(v) <= kFinishStopSpeedMps) {
    completed_ = true;
    lap_time_s_ = elapsed_s_;
  }
}

Verdict RaceArbiter::verdict() const {
  return {completed_, lap_time_s_, infraction_};
}

}
