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

[[nodiscard]] inline const char* infraction_name(Infraction infraction) {
  switch (infraction) {
    case Infraction::kObstacleCollision:
      return "OBSTACLE COLLISION";
    case Infraction::kOutOfBounds:
      return "OUT OF BOUNDS";
    case Infraction::kLineLostTimeout:
      return "LINE LOST TIMEOUT";
    case Infraction::kTimeLimitExceeded:
      return "TIME LIMIT EXCEEDED";
    case Infraction::kNone:
    default:
      return "NONE";
  }
}

struct Verdict {
  bool completed = false;
  float lap_time_s = 0.0f;
  Infraction infraction = Infraction::kNone;
};

// Giám sát luật thi đấu (Single Source of Truth), độc lập với niềm tin robot.
class TrackRules {
public:
  void reset(const TrackManifest& manifest);
  void check(float x, float y, float v, bool line_detected, float dt);
  [[nodiscard]] Verdict verdict() const;

private:
  const TrackManifest* manifest_ = nullptr;
  float elapsed_s_ = 0.0f;
  float line_lost_s_ = 0.0f;
  bool completed_ = false;
  float lap_time_s_ = 0.0f;
  Infraction infraction_ = Infraction::kNone;
};

}  // namespace lf::sim
