#pragma once

#include <cstdint>

#include "track_asset.hpp"
#include "world_physics.hpp"

namespace lf::sim {

enum class VerdictKind : std::uint8_t { kPending, kCompleted, kDnf };

struct RefereeResult {
  VerdictKind verdict = VerdictKind::kPending;
  std::uint64_t elapsed_us = 0;
  std::uint8_t run = 0;
};

// Judges one run from world poses. It never sees controller status: a run is
// completed or disqualified by what happened on the track.
class Referee {
public:
  explicit Referee(const Calibration& calibration = simulated_calibration()) noexcept
    : half_length_mm_(calibration.half_length_mm), half_width_mm_(calibration.half_width_mm) {}
  void reset() noexcept;
  void start_run(std::uint64_t now_us, std::uint8_t run) noexcept;
  [[nodiscard]] RefereeResult check(const TrackAsset& track, const Pose& current,
                                    std::uint64_t time_us) noexcept;

private:
  float half_length_mm_;
  float half_width_mm_;
  std::uint64_t start_us_ = 0;
  std::uint8_t run_ = 0;
  bool started_ = false;
  RefereeResult result_;
};

}  // namespace lf::sim
