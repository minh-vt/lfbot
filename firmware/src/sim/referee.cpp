#include "referee.hpp"

#include <array>
#include <cmath>
#include <cstddef>

#include "robot_geometry.hpp"

namespace lf::sim {
namespace {

constexpr std::uint64_t kRunLimitUs = 180000000U;

// Gate contact is a footprint overlap, not corner containment: the gate can
// cut through the chassis without containing any of its corners. The gate is
// axis-aligned per the fixture contract; its normal is a separating axis.
// Physics substeps the pose, and the gate is thicker than one tick of travel in
// the simulated envelope, so a per-tick overlap test does not skip a crossing.
bool footprint_touches_gate(const TrackAsset& track, const Pose& pose, float half_length,
                            float half_width) noexcept {
  const float gate_x = 0.5F * (track.finish_min_x_mm + track.finish_max_x_mm);
  const float gate_y = 0.5F * (track.finish_min_y_mm + track.finish_max_y_mm);
  const float gate_half_x = 0.5F * (track.finish_max_x_mm - track.finish_min_x_mm);
  const float gate_half_y = 0.5F * (track.finish_max_y_mm - track.finish_min_y_mm);
  const float dx = pose.x_mm - gate_x;
  const float dy = pose.y_mm - gate_y;
  const float c = std::cos(pose.yaw_rad);
  const float s = std::sin(pose.yaw_rad);
  const std::array<float, 8> axes = {c, s, -s, c, 1.0F, 0.0F, 0.0F, 1.0F};
  for (std::size_t i = 0; i < axes.size(); i += 2) {
    const float ax = axes[i];
    const float ay = axes[i + 1];
    const float robot_radius =
      half_length * std::abs(ax * c + ay * s) + half_width * std::abs(-ax * s + ay * c);
    const float gate_radius = gate_half_x * std::abs(ax) + gate_half_y * std::abs(ay);
    if (std::abs(dx * ax + dy * ay) > robot_radius + gate_radius) {
      return false;
    }
  }
  return true;
}

}  // namespace

void Referee::reset() noexcept {
  start_us_ = 0;
  run_ = 0;
  started_ = false;
  result_ = {};
}

void Referee::start_run(std::uint64_t now_us, std::uint8_t run) noexcept {
  start_us_ = now_us;
  run_ = run;
  started_ = true;
  result_ = {};
}

RefereeResult Referee::check(const TrackAsset& track, const Pose& current,
                             std::uint64_t time_us) noexcept {
  if (!started_ || result_.verdict != VerdictKind::kPending) {
    return result_;
  }
  const std::uint64_t elapsed = time_us >= start_us_ ? time_us - start_us_ : 0;
  if (footprint_touches_gate(track, current, half_length_mm_, half_width_mm_)) {
    result_ = {VerdictKind::kCompleted, elapsed, run_};
  } else if (elapsed >= kRunLimitUs) {
    result_ = {VerdictKind::kDnf, elapsed, run_};
  }
  return result_;
}

}  // namespace lf::sim
