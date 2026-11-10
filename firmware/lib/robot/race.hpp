#pragma once

#include <array>
#include <cstddef>
#include <optional>

#include "robot_data.hpp"
#include "route.hpp"

namespace lf {

// Internal to the core package: platforms drive through Robot, not Race.
class Race {
public:
  Race(const RaceGeometry& geometry, const RaceTuning& tuning) noexcept;
  Race(const Race&) = delete;
  Race& operator=(const Race&) = delete;
  Race(Race&&) = delete;
  Race& operator=(Race&&) = delete;
  ~Race() = default;

  [[nodiscard]] MotorDemand drive(const SensorReadings& sensors, RaceRequest request) noexcept;
  [[nodiscard]] const RaceStatus& status() const noexcept { return status_; }

  // After the referee has judged: stop driving, keep the recorded memory.
  void finish_run() noexcept;

private:
  static constexpr std::size_t kRegionCapacity = Route::kObstacleCapacity + 1U;
  static constexpr std::size_t kSpeedLevels = 8U;

  // A body-frame exclusion disk that already includes the robot's extent, so the
  // robot can be treated as a point. Bounded set: map regions plus the live hit.
  struct Region {
    Point2 centre;
    float radius = 0.0F;
  };

  [[nodiscard]] float measure_line_offset(const SensorReadings& sensors) noexcept;
  void update_line_state(const SensorReadings& sensors) noexcept;
  void update_progress(const SensorReadings& sensors) noexcept;
  // False when the forward range cannot be observed at all; unknown is not clear.
  [[nodiscard]] static bool ahead_bounded(const SensorReadings& sensors) noexcept;
  [[nodiscard]] float slip_limit(float curvature) const noexcept;
  [[nodiscard]] bool jammed(const SensorReadings& sensors, float speed) noexcept;
  // One in-place revolution for the line. False when the sweep is spent.
  [[nodiscard]] bool search_lost_line(float& yaw_rate) noexcept;
  [[nodiscard]] float apply_memory(float speed) noexcept;
  [[nodiscard]] bool accept_request(RaceRequest request) noexcept;

  [[nodiscard]] float line_curvature() const noexcept;
  [[nodiscard]] std::size_t collect_regions(
    const SensorReadings& sensors, std::array<Region, kRegionCapacity>& regions) const noexcept;
  [[nodiscard]] Point2 steer_target(std::size_t count,
                                    const std::array<Region, kRegionCapacity>& regions,
                                    Point2 line_target) const noexcept;
  // Smallest body-to-region gap over the driven arc; negative means overlap.
  [[nodiscard]] static float swept_clearance(float curvature, float distance,
                                             const std::array<Region, kRegionCapacity>& regions,
                                             std::size_t count) noexcept;
  [[nodiscard]] float feasible_speed(float ceiling, float curvature,
                                     const std::array<Region, kRegionCapacity>& regions,
                                     std::size_t count) const noexcept;
  [[nodiscard]] float stopping_distance(float speed) const noexcept;
  // Whether a region sits in the corridor the line arc would drive through, so a
  // bypass starts before the line arc is already dead.
  [[nodiscard]] bool region_blocking(
    std::size_t count, const std::array<Region, kRegionCapacity>& regions) const noexcept;
  // One tick of a body-right motion about the nearest region ahead. False when no
  // arc clears or the search has spent its distance budget.
  [[nodiscard]] bool plan_bypass(const std::array<Region, kRegionCapacity>& regions,
                                 std::size_t count, float interval, float& speed,
                                 float& yaw_rate) noexcept;
  // The motion for this tick: the line arc, or a bypass arc when a region blocks
  // it. False when no motion is honest and the run must stop.
  [[nodiscard]] bool choose_action(const SensorReadings& sensors,
                                   const std::array<Region, kRegionCapacity>& regions,
                                   std::size_t count, float planned, float curvature, float& speed,
                                   float& yaw_rate) noexcept;
  // Ramp the wheel pair toward (speed, yaw_rate) at the declared drive limit.
  [[nodiscard]] std::array<float, 2> ramp_wheels(float speed, float yaw_rate,
                                                 float interval) noexcept;
  // Whether the motion this wheel pair actually produces clears every region.
  [[nodiscard]] bool wheels_clear(const std::array<float, 2>& wheels,
                                  const std::array<Region, kRegionCapacity>& regions,
                                  std::size_t count) const noexcept;
  [[nodiscard]] MotorDemand emit_wheels(const std::array<float, 2>& wheels) noexcept;

  RaceGeometry geometry_;
  RaceTuning tuning_;
  Route route_;
  RaceStatus status_;
  float offset_ = 0.0F;
  // The offset above is only a measurement while a lit sensor is under the line;
  // otherwise it is the last sighting, which is evidence about the past.
  bool line_live_ = false;
  // Nothing observed yet in this run is not the same as a line that has left the
  // array: at the start the array may simply not have reported, and a stop would
  // end the run before the first sample arrives.
  bool seen_line_ = false;
  bool tracking_ = false;
  float progress_ = 0.0F;
  float run_start_progress_ = 0.0F;
  float stall_time_ = 0.0F;      // T0 while commanding forward with no wheel progress
  float stale_ir_time_ = 0.0F;   // T0 without a fresh IR sample
  float edge_side_ = 0.0F;       // sign of the array edge the line was last seen leaving
  float search_heading_ = 0.0F;  // estimated heading when the line was lost
  bool has_search_heading_ = false;
  bool ir_alive_ = false;     // a fresh IR sample has arrived in this run
  bool pivot_spent_ = false;  // this loss of sight has used its rotation budget
  // A bypass is the only motion that still makes progress when the line arc is
  // blocked. It is not a stored detour: every tick re-picks the arc from the
  // regions as they are now.
  bool bypassing_ = false;
  float bypass_travel_ = 0.0F;    // L0 driven since the bypass started
  unsigned line_sightings_ = 0U;  // consecutive fresh line observations
  float distance_step_ = 0.0F;
  float influence_ = 0.0F;  // T0-derived reach of the exclusion field, in L0
  std::array<float, 2> wheel_reference_{};
  bool has_reference_ = false;
  std::array<float, 64> run_one_limit_{};
  std::size_t run_one_limit_size_ = 0;
  bool has_offset_ = false;
};

}  // namespace lf
