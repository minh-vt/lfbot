#pragma once

#include <cstdint>

#include "sim_config.hpp"

namespace lf::sim {

enum class Infraction : std::uint8_t {
  kNone = 0,
  kObstacleCollision,
  kOutOfBounds,
  kLineLostTimeout,
  kTimeLimitExceeded,
};

struct Verdict {
  bool completed = false;
  float lap_time_s = 0.0f;
  Infraction infraction = Infraction::kNone;
};

// Trọng tài ground-truth, độc lập với niềm tin của robot.
class RaceArbiter {
public:
  void reset(const TrackManifest& manifest);
  void observe(float x, float y, float v, bool line_detected, float dt);
  [[nodiscard]] Verdict verdict() const;

private:
  TrackManifest manifest_;
  float elapsed_s_ = 0.0f;
  float line_lost_s_ = 0.0f;
  bool completed_ = false;
  float lap_time_s_ = 0.0f;
  Infraction infraction_ = Infraction::kNone;
};

}  // namespace lf::sim
