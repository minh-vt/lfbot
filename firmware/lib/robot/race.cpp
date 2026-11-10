#include "race.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace lf {
namespace {

// Dimensionless sensor/domain constants, not control gains. The commanded action
// is a wheel pair, (speed, yaw_rate); the line term supplies one arc, the map adds
// a tangential circulation, and an exact swept-arc test limits what that pair may
// express.
constexpr float kLineThreshold = 0.35F;    // IR normalized "line present"
constexpr float kSensorFaultLimit = 1.5F;  // T0 without a fresh IR sample before fault
constexpr float kStallLimit = 1.0F;        // T0 commanding forward with no wheel progress
constexpr float kCommandValidity = 2.0F;
constexpr float kMemoryBucket = 0.25F;
constexpr float kCrawlSpeed = 0.12F;  // floor when the path ahead or the line is unknown
// The array stands 59 mm ahead and 62 mm wide, so a flank steeper than about 28
// degrees never enters the field of view at any speed. Rotating in place is the
// only motion that can bring it back, and half the pair at full opposite duty is
// the fastest rotation the wheels allow. The budget bounds the search in time and
// spends no distance; it ends when the line is seen again.
constexpr float kPivotRate = 2.0F;
// One full revolution of the estimated heading, whatever the wheel speed: the
// budget is geometry, not a tunable time. A sweep that has turned a full circle
// without seeing the line has covered everything the array can reach.
constexpr float kSearchTurns = 6.28318530717958647692F;
// A bypass is a bounded motion about one obstacle, re-planned every tick: the
// robot drives one tick of the first arc that clears and re-reads the regions.
constexpr float kBypassSpeed = 0.30F;  // V0; slow enough to re-plan each tick
constexpr float kBypassBudget = 8.0F;  // L0 of travel before it gives up
// The pass-by point sits outside the largest object the rules allow, not just
// outside the surface patch that was measured: a single range reading never sees
// the far side of the brick, so a pass planned from the patch alone would clip it.
constexpr float kPassMargin = 0.10F;          // L0 beyond the declared brick envelope
constexpr float kCurlCurvature = 1.0F;        // 1/L0 back toward the left after passing
constexpr float kHardestCurvature = 2.0F;     // 1/L0; no chassis holds a tighter arc
constexpr unsigned kReacquireSightings = 3U;  // sightings that make it the line
// Closed-loop stability, not a taste constant: pure pursuit over a lookahead L has
// an undamped natural frequency of sqrt(2)*v/L, and a steering loop faster than
// the actuator's own bandwidth follows its own lag instead of the line. Keeping
// that frequency under half of 1/motor_response bounds the speed the array may
// carry; the number falls out of geometry and the drive model.
constexpr float kLoopToActuatorRatio = 0.16F;
constexpr float kPi = 3.14159265358979323846F;
constexpr float kArcEpsilon = 1e-4F;
constexpr float kFieldEpsilon = 1e-4F;
constexpr std::uint8_t kMaxRuns = 2;
constexpr std::size_t kProjectionPasses = 4U;
// Freshness in T0. The simulator's ToF updates every ~0.07 T0, so a stale
// reading is a fault, not a normal gap between acquisitions. These are
// simulation-proposal budgets; hardware limits must be measured.
constexpr float kMaxIrAge = 0.10F;
constexpr float kMaxTofAge = 0.30F;
constexpr float kMaxWheelAge = 0.10F;

bool is_fresh(const SensorObservation& sample, float max_age) noexcept {
  return sample.status == ReadingStatus::kValid && sample.age <= max_age;
}

// Geometric point an arc of the given curvature reaches after `length`, in the
// body frame. Pure pursuit on this point returns exactly that curvature, so a
// line-following law and the obstacle field can share one tracker.
Point2 arc_point(float curvature, float length) noexcept {
  if (std::abs(curvature) < kArcEpsilon) {
    return {length, 0.0F};
  }
  const float angle = curvature * length;
  return {std::sin(angle) / curvature, (1.0F - std::cos(angle)) / curvature};
}

bool arc_contains(float sweep, float relative) noexcept {
  return sweep >= 0.0F ? (relative >= 0.0F && relative <= sweep)
                       : (relative <= 0.0F && relative >= sweep);
}

}  // namespace

Race::Race(const RaceGeometry& geometry, const RaceTuning& tuning) noexcept
  : geometry_(geometry), tuning_(tuning), route_(geometry, tuning.robot) {}

float Race::measure_line_offset(const SensorReadings& sensors) noexcept {
  // Coverage-weighted lateral moment, not a centroid: dividing by the number of
  // lit sensors would dilute a near-perpendicular line that covers one whole
  // side, making the controller under-turn exactly at sharp corners. A fixed
  // scale (the sensor span) keeps the ranking: more lit sensors on one side =
  // stronger signal.
  float lateral_sum = 0.0F;
  bool any = false;
  bool low_edge = false;
  bool high_edge = false;
  for (std::size_t i = 0; i < sensors.ir.size(); ++i) {
    const auto& sample = sensors.ir[i];
    if (!is_fresh(sample, kMaxIrAge) || sample.value < kLineThreshold) {
      continue;
    }
    lateral_sum += sample.value * geometry_.ir_position[i].y;
    any = true;
    low_edge = low_edge || i == 0U;
    high_edge = high_edge || i + 1U == sensors.ir.size();
  }
  if (any) {
    has_offset_ = true;
    line_live_ = true;
    seen_line_ = true;
    // Which way the line leaves the array is read from the outermost sensors, not
    // from the moment: the moment shrinks as coverage shrinks, so it cannot say
    // "this line is heading off the side". Only one edge lit means the line is
    // past that edge; both edges lit is a wide patch with no side to prefer.
    const float low_sign = geometry_.ir_position[0].y < 0.0F ? -1.0F : 1.0F;
    if (low_edge && !high_edge) {
      edge_side_ = low_sign;
    } else if (high_edge && !low_edge) {
      edge_side_ = -low_sign;
    } else {
      edge_side_ = 0.0F;
    }
    // The moment is a fraction of the sensor span: the whole array lit on one
    // side is 0.5, not a distance. Convert it to a lateral error in the same
    // length unit as the lookahead, otherwise the curvature term is off by the
    // span-to-lookahead ratio and every correction saturates.
    const float fraction = lateral_sum / geometry_.sensor_span_sum;
    return 2.0F * fraction * geometry_.sensor_span_half;
  }
  line_live_ = false;
  return has_offset_ ? offset_ : 0.0F;
}

void Race::update_line_state(const SensorReadings& sensors) noexcept {
  offset_ = measure_line_offset(sensors);
  const bool sees_line =
    std::any_of(sensors.ir.begin(), sensors.ir.end(), [](const SensorObservation& sample) {
      return is_fresh(sample, kMaxIrAge) && sample.value >= kLineThreshold;
    });
  tracking_ = sees_line;
  if (sees_line) {
    pivot_spent_ = false;
  }
}

void Race::update_progress(const SensorReadings& sensors) noexcept {
  route_.record_readings(sensors);
  distance_step_ = 0.0F;
  if (!route_.has_position() || !is_fresh(sensors.wheel_positions[0], kMaxWheelAge) ||
      !is_fresh(sensors.wheel_positions[1], kMaxWheelAge)) {
    return;
  }
  progress_ = std::max(progress_, route_.locate_robot().wheel_position);
  const auto motion = route_.motion();
  distance_step_ = motion.distance;
}

bool Race::ahead_bounded(const SensorReadings& sensors) noexcept {
  // A range-limit no-hit is a bounded observation along the sampled rays, not a
  // guarantee that the whole field of view is empty. It is still different from
  // no observation at all: only the latter forbids cruising.
  const auto& tof = sensors.tof[0];
  return (tof.status == ReadingStatus::kValid || tof.status == ReadingStatus::kNoReturn) &&
         tof.age <= kMaxTofAge;
}

bool Race::jammed(const SensorReadings& sensors, float speed) noexcept {
  if (speed <= 0.0F) {
    stall_time_ = 0.0F;
    return false;
  }
  const bool wheels_fresh = is_fresh(sensors.wheel_positions[0], kMaxWheelAge) &&
                            is_fresh(sensors.wheel_positions[1], kMaxWheelAge);
  if (!wheels_fresh || distance_step_ > 1e-4F) {
    stall_time_ = 0.0F;
    return false;
  }
  stall_time_ += sensors.elapsed_time;
  return stall_time_ > kStallLimit;
}

float Race::slip_limit(float curvature) const noexcept {
  const auto& axis = tuning_.robot.slip_curvature;
  const auto& limit = tuning_.robot.slip_limit;
  if (curvature <= axis.front()) {
    return limit.front();
  }
  if (curvature >= axis.back()) {
    return limit.back();
  }
  for (std::size_t i = 1; i < axis.size(); ++i) {
    if (curvature <= axis[i]) {
      const float t = (curvature - axis[i - 1]) / (axis[i] - axis[i - 1]);
      return limit[i - 1] + t * (limit[i] - limit[i - 1]);
    }
  }
  return limit.back();
}

float Race::apply_memory(float speed) noexcept {
  const float run_progress = progress_ - run_start_progress_;
  const auto raw = run_progress / kMemoryBucket;
  const auto bucket =
    static_cast<std::size_t>(std::min(raw, static_cast<float>(run_one_limit_.size() - 1U)));
  if (status_.runs_used >= kMaxRuns) {
    // Run two may only exploit speeds already proven usable in run one.
    if (bucket < run_one_limit_size_ && run_one_limit_[bucket] > 0.0F) {
      return std::min(speed, run_one_limit_[bucket]);
    }
    return speed;
  }
  // Record the best speed this progress actually reached, not the smallest: a
  // single slow tick (a sharp feature, a search) must not cap the whole bucket for
  // the rest of the run.
  run_one_limit_[bucket] = std::max(run_one_limit_[bucket], speed);
  run_one_limit_size_ = std::max(run_one_limit_size_, bucket + 1U);
  return speed;
}

float Race::line_curvature() const noexcept {
  // Pure pursuit: the forward IR array is the lookahead, so the coverage-weighted
  // lateral moment is the goal offset at distance L; the curvature is geometric,
  // never a hand-tuned gain. It is only geometric while the line is under the
  // array. An array that sees white is a measurement of absence, not a stale
  // reading: steering toward the last sighting would keep turning on evidence
  // that has already expired and spin the robot off the track.
  if (!line_live_) {
    return 0.0F;
  }
  const float lookahead = geometry_.lookahead;
  const float y = offset_;
  return 2.0F * y / (lookahead * lookahead + y * y);
}

std::size_t Race::collect_regions(const SensorReadings& sensors,
                                  std::array<Region, kRegionCapacity>& regions) const noexcept {
  std::size_t count = 0;
  // Remembered regions survive leaving the field of view; that memory is what
  // stops the "sees brick, loses brick, turns back into it" limit cycle. They are
  // only usable while the pose gauge still places them well enough to certify a
  // gap: a record that could be anywhere is not a smaller obstacle, it is no
  // knowledge at all, and the live range is then the only authority.
  const float usable_uncertainty = 0.5F * geometry_.body_radius;
  if (route_.has_position()) {
    const auto& pose = route_.locate_robot();
    const float cosine = std::cos(pose.heading);
    const float sine = std::sin(pose.heading);
    for (std::size_t i = 0; i < route_.obstacle_size() && count < kRegionCapacity; ++i) {
      const auto obstacle = route_.locate_obstacle(i, pose);
      if (obstacle.uncertainty > usable_uncertainty) {
        continue;
      }
      const float dx = obstacle.centre.x - pose.position.x;
      const float dy = obstacle.centre.y - pose.position.y;
      regions[count++] = {
        {cosine * dx + sine * dy, -sine * dx + cosine * dy},
        obstacle.radius + geometry_.body_radius,
      };
    }
  }
  const auto& tof = sensors.tof[0];
  if (is_fresh(tof, kMaxTofAge) && count < kRegionCapacity) {
    const float range = tof.value;
    // A scalar range over a finite field of view constrains a patch of surface
    // points, not the brick. The body radius is the only dilation: it is the
    // robot's own size, which is what turns a region into a keep-out test.
    regions[count++] = {
      {geometry_.tof_forward + range, 0.0F},
      observed_patch_radius(geometry_, range) + geometry_.body_radius,
    };
  }
  return count;
}

Point2 Race::steer_target(std::size_t count, const std::array<Region, kRegionCapacity>& regions,
                          Point2 line_target) const noexcept {
  const float reach = std::hypot(line_target.x, line_target.y);
  Point2 target = line_target;
  std::array<Point2, kRegionCapacity> normals{};
  std::array<float, kRegionCapacity> gaps{};
  std::size_t active = 0;
  for (std::size_t i = 0; i < count; ++i) {
    const auto& region = regions[i];
    const float rho = std::hypot(region.centre.x, region.centre.y);
    if (rho < kArcEpsilon) {
      continue;
    }
    const float gap = rho - region.radius;
    if (gap >= influence_) {
      continue;
    }
    // Smoothstep on the gap: full circulation at contact, zero at the reach.
    const float z = std::clamp(gap / influence_, 0.0F, 1.0F);
    const float weight = 1.0F - z * z * (3.0F - 2.0F * z);
    // Outward normal of the exclusion region, then +90 degrees about it. Facing
    // the region this tangent is body-right, independent of screen heading.
    const Point2 normal{-region.centre.x / rho, -region.centre.y / rho};
    target.x += weight * reach * -normal.y;
    target.y += weight * reach * normal.x;
    normals[active] = normal;
    gaps[active] = gap;
    ++active;
  }
  // Bound every relevant region at once: the target must not aim into a region
  // by more than the current gap allows. A few bounded passes handle corners
  // where two regions constrain the same direction.
  for (std::size_t pass = 0; pass < kProjectionPasses; ++pass) {
    for (std::size_t i = 0; i < active; ++i) {
      const float inward = normals[i].x * target.x + normals[i].y * target.y;
      const float limit = -(std::max(gaps[i], 0.0F) / influence_) * reach;
      if (inward < limit) {
        target.x += (limit - inward) * normals[i].x;
        target.y += (limit - inward) * normals[i].y;
      }
    }
  }
  return target;
}

float Race::swept_clearance(float curvature, float distance,
                            const std::array<Region, kRegionCapacity>& regions,
                            std::size_t count) noexcept {
  float worst = 0.0F;
  bool first = true;
  for (std::size_t i = 0; i < count; ++i) {
    const auto& region = regions[i];
    float nearest = 0.0F;
    if (std::abs(curvature) < kArcEpsilon) {
      // Straight sweep: distance from the region centre to the segment.
      const float along = std::clamp(region.centre.x, 0.0F, distance);
      nearest = std::hypot(region.centre.x - along, region.centre.y);
    } else {
      const float radius = 1.0F / curvature;
      const Point2 centre{0.0F, radius};
      const float dx = region.centre.x - centre.x;
      const float dy = region.centre.y - centre.y;
      const float offset = std::atan2(dy, dx);
      const float start = std::atan2(-centre.y, -centre.x);
      const float relative = std::remainder(offset - start, 2.0F * kPi);
      if (arc_contains(curvature * distance, relative)) {
        // Bearing falls inside the sweep: the closest point is on the arc.
        nearest = std::abs(std::hypot(dx, dy) - std::abs(radius));
      } else {
        const Point2 end = arc_point(curvature, distance);
        nearest = std::min(std::hypot(region.centre.x, region.centre.y),
                           std::hypot(region.centre.x - end.x, region.centre.y - end.y));
      }
    }
    const float clearance = nearest - region.radius;
    worst = first ? clearance : std::min(worst, clearance);
    first = false;
  }
  return worst;
}

float Race::stopping_distance(float speed) const noexcept {
  const float delay = geometry_.motor_response + kMaxWheelAge;
  const float magnitude = std::abs(speed);
  return magnitude * delay + magnitude * magnitude / (2.0F * geometry_.brake_deceleration);
}

float Race::feasible_speed(float ceiling, float curvature,
                           const std::array<Region, kRegionCapacity>& regions,
                           std::size_t count) const noexcept {
  if (count == 0U) {
    return ceiling;
  }
  // Descending candidates: take the fastest speed whose sweep and braking arc
  // still clear every region. No feasible level means the map already conflicts
  // with the footprint, so stopping is the only honest command.
  for (std::size_t level = 0; level < kSpeedLevels; ++level) {
    const float speed =
      ceiling * static_cast<float>(kSpeedLevels - level) / static_cast<float>(kSpeedLevels);
    if (swept_clearance(curvature, stopping_distance(speed), regions, count) >= 0.0F) {
      return speed;
    }
  }
  return 0.0F;
}

bool Race::region_blocking(std::size_t count,
                           const std::array<Region, kRegionCapacity>& regions) const noexcept {
  for (std::size_t i = 0; i < count; ++i) {
    const Region& region = regions[i];
    // Centre x is the body forward axis, so the reach to the region's near surface
    // is centre minus radius; body extent is already inside the radius.
    if (region.centre.x > 0.0F && std::abs(region.centre.y) <= region.radius &&
        region.centre.x - region.radius <= influence_) {
      return true;
    }
  }
  return false;
}

bool Race::plan_bypass(const std::array<Region, kRegionCapacity>& regions, std::size_t count,
                       float interval, float& speed, float& yaw_rate) noexcept {
  // The region to get past is the nearest one that is not already behind the axle.
  std::size_t pick = kRegionCapacity;
  float nearest = 0.0F;
  for (std::size_t i = 0; i < count; ++i) {
    const Region& region = regions[i];
    if (region.centre.x + region.radius <= 0.0F) {
      continue;
    }
    const float reach = std::hypot(region.centre.x, region.centre.y);
    if (pick == kRegionCapacity || reach < nearest) {
      pick = i;
      nearest = reach;
    }
  }

  // Softest first: the arc that reaches a point clear of the region, then harder
  // body-right arcs in case the sweep rejects the soft one. The last entry turns
  // away to open room for a better look, which spends the budget slowly and never
  // claims the arc was known to be enough.
  std::array<float, 4> arcs{};
  std::size_t arc_count = 0;
  if (pick != kRegionCapacity) {
    const Region& region = regions[pick];
    const float envelope =
      std::max(region.radius, geometry_.obstacle_radius + geometry_.body_radius);
    const float pass = envelope + kPassMargin;
    const float reach = std::max(nearest, kArcEpsilon);
    // Body-right normal of the bearing to the region: (x, y) -> (y, -x), because
    // the body frame counts y to the left.
    const Point2 target{
      region.centre.x + pass * (region.centre.y / reach),
      region.centre.y - pass * (region.centre.x / reach),
    };
    if (target.x > kArcEpsilon) {
      const float chord = target.x * target.x + target.y * target.y;
      const float aim = 2.0F * target.y / chord;
      arcs[arc_count++] = aim;
      arcs[arc_count++] = std::clamp(1.5F * aim, -kHardestCurvature, kHardestCurvature);
      arcs[arc_count++] = std::clamp(3.0F * aim, -kHardestCurvature, kHardestCurvature);
    }
  }
  arcs[arc_count++] = kCurlCurvature;

  for (std::size_t i = 0; i < arc_count; ++i) {
    const float curvature = arcs[i];
    const float ceiling = std::min(kBypassSpeed, 1.0F / (1.0F + 0.5F * std::abs(curvature)));
    const float trial = feasible_speed(ceiling, curvature, regions, count);
    if (trial > 0.0F) {
      speed = trial;
      yaw_rate = trial * curvature;
      bypass_travel_ += trial * interval;
      return bypass_travel_ <= kBypassBudget;
    }
  }
  speed = 0.0F;
  yaw_rate = 0.0F;
  return false;
}

bool Race::choose_action(const SensorReadings& sensors,
                         const std::array<Region, kRegionCapacity>& regions, std::size_t count,
                         float planned, float curvature, float& speed, float& yaw_rate) noexcept {
  // A line that has left the array next to a region is the obstacle case, not the
  // sharp-corner case: turning in place there only faces the brick again. Sighting
  // are counted so a bypass ends on the line, not on one lit sample, and only when
  // the line is under the middle of the array: a graze at the edge is what the
  // bypass just turned away from.
  if (line_live_ && std::abs(offset_) <= 0.5F * geometry_.sensor_span_half) {
    if (line_sightings_ < 255U) {
      ++line_sightings_;
    }
  } else {
    line_sightings_ = 0U;
  }
  if (bypassing_ && line_sightings_ >= kReacquireSightings) {
    bypassing_ = false;
    bypass_travel_ = 0.0F;
  }

  if (bypassing_ || planned <= 0.0F || region_blocking(count, regions)) {
    // Stopping at the first obstacle would end every run at the brick, so the robot
    // drives a checked body-right arc instead. plan_bypass reports false only when
    // no arc clears or the budget is spent, and stopping is then the honest command.
    if (!plan_bypass(regions, count, sensors.elapsed_time, speed, yaw_rate)) {
      return false;
    }
    bypassing_ = true;
    return true;
  }
  // Remembering a proven speed belongs to the speed the robot actually drives, not
  // to the ceiling: the ceiling carries a one-tick steering transient, and feeding
  // that into the memory would let a single sharp feature cap the whole run.
  speed = apply_memory(planned);
  yaw_rate = speed * curvature;

  // The line law only decides the arc while a lit sensor is under the line. When
  // the array sees white, the last sighting says which way the line was heading;
  // if it was far off centre the line has left through the side of the array,
  // which is what a turn steeper than the field of view does. Rotating in place
  // then recovers it without committing any distance, and the search ends as soon
  // as the line is seen again.
  if (!line_live_ && !seen_line_) {
    // A blind start: the run has not seen the line at all, so there is no side to
    // search and no evidence to give up on. Creep straight ahead at the floor
    // speed until the array reports something.
    speed = std::min(speed, kCrawlSpeed);
    yaw_rate = 0.0F;
    return true;
  }
  if (!line_live_) {
    // A full revolution that found nothing means the line is outside the circle
    // the array sweeps. Creeping on would be distance committed without evidence,
    // which is how the robot leaves the track and still reports "running".
    if (!search_lost_line(yaw_rate)) {
      return false;
    }
    speed = 0.0F;
    return true;
  }
  has_search_heading_ = false;
  return true;
}

std::array<float, 2> Race::ramp_wheels(float speed, float yaw_rate, float interval) noexcept {
  // The plan is a wheel *speed*, the actuator takes *duty*. At steady state the
  // declared drive model maps duty to speed, so that is the whole conversion; the
  // reference is accelerated at the declared limit so the demand stays inside what
  // the drive can follow. Adding the model's derivative term here would amplify
  // per-tick line noise into duty swings instead of tracking a planned
  // trajectory, so it is deliberately absent.
  std::array<float, 2> wheels{};
  const float step = geometry_.drive_acceleration * std::max(interval, 0.0F);
  for (std::size_t i = 0; i < wheels.size(); ++i) {
    const float side = i == 0U ? -1.0F : 1.0F;
    const float target = speed + 0.5F * side * yaw_rate;
    wheels[i] = has_reference_
                  ? wheel_reference_[i] + std::clamp(target - wheel_reference_[i], -step, step)
                  : target;
  }
  return wheels;
}

bool Race::wheels_clear(const std::array<float, 2>& wheels,
                        const std::array<Region, kRegionCapacity>& regions,
                        std::size_t count) const noexcept {
  if (count == 0U) {
    return true;
  }
  // The arc this pair actually produces, not the arc that was planned: ramping
  // from the previous reference changes the ratio of the two wheels, and that
  // transition is what the geometry has to survive.
  const float speed = 0.5F * (wheels[0] + wheels[1]);
  const float yaw_rate = wheels[1] - wheels[0];
  if (std::abs(speed) < kArcEpsilon) {
    // Rotating in place sweeps the footprint only, and every region already
    // includes the robot's extent, so the current pose must simply be clear.
    for (std::size_t i = 0; i < count; ++i) {
      if (std::hypot(regions[i].centre.x, regions[i].centre.y) < regions[i].radius) {
        return false;
      }
    }
    return true;
  }
  return swept_clearance(yaw_rate / speed, stopping_distance(speed), regions, count) >= 0.0F;
}

MotorDemand Race::emit_wheels(const std::array<float, 2>& wheels) noexcept {
  // Saturate the pair, never one wheel alone. Scaling both by the same factor
  // preserves the ratio, so the curvature that was checked is the curvature
  // driven; only the speed falls, which shortens the sweep.
  const float peak = std::max(std::abs(wheels[0]), std::abs(wheels[1]));
  const float scale = peak > 1.0F ? 1.0F / peak : 1.0F;
  for (std::size_t i = 0; i < wheels.size(); ++i) {
    wheel_reference_[i] = wheels[i] * scale;
  }
  MotorDemand demand;
  demand.left_duty = wheel_reference_[0] / geometry_.duty_scale;
  demand.right_duty = wheel_reference_[1] / geometry_.duty_scale;
  demand.valid_for_time = kCommandValidity;
  const bool moving =
    std::abs(wheel_reference_[0]) > kArcEpsilon || std::abs(wheel_reference_[1]) > kArcEpsilon;
  demand.mode = moving ? MotorMode::kDrive : MotorMode::kBrake;
  return demand;
}

void Race::finish_run() noexcept {
  status_.phase = RacePhase::kDone;
  // An operator abort is a real stop reason and must survive the transition.
  if (status_.reason != StopReason::kAborted) {
    status_.reason = StopReason::kNone;
  }
}

bool Race::accept_request(RaceRequest request) noexcept {
  if (request == RaceRequest::kReset) {
    status_ = {};
    offset_ = 0.0F;
    tracking_ = false;
    stale_ir_time_ = 0.0F;
    ir_alive_ = false;
    progress_ = 0.0F;
    run_start_progress_ = 0.0F;
    stall_time_ = 0.0F;
    route_.reset();
    run_one_limit_.fill(0.0F);
    run_one_limit_size_ = 0;
    has_offset_ = false;
    seen_line_ = false;
    bypassing_ = false;
    bypass_travel_ = 0.0F;
    line_sightings_ = 0U;
    wheel_reference_.fill(0.0F);
    has_reference_ = false;
    influence_ = 0.0F;
    return false;
  }
  if (request == RaceRequest::kAbort) {
    status_.phase = RacePhase::kDone;
    status_.reason = StopReason::kAborted;
    return false;
  }
  if (status_.phase == RacePhase::kFault) {
    return false;
  }
  if (request == RaceRequest::kStart) {
    if (status_.phase == RacePhase::kRunning) {
      return true;
    }
    if (status_.runs_used >= kMaxRuns) {
      return false;
    }
    ++status_.runs_used;
    status_.phase = RacePhase::kRunning;
    status_.reason = StopReason::kNone;
    offset_ = 0.0F;
    run_start_progress_ = progress_;
    tracking_ = false;
    stale_ir_time_ = 0.0F;
    ir_alive_ = false;
    stall_time_ = 0.0F;
    route_.prepare_run();
    has_offset_ = false;
    seen_line_ = false;
    bypassing_ = false;
    bypass_travel_ = 0.0F;
    line_sightings_ = 0U;
    // The reposition puts the robot back at the start; the wheel-speed reference
    // is a live state of this run and must not carry over.
    wheel_reference_.fill(0.0F);
    has_reference_ = false;
    influence_ = 0.0F;
    return true;
  }
  if (status_.phase != RacePhase::kRunning) {
    // A finished run keeps the reason it stopped for; only an idle robot has no
    // accepted request to act on.
    if (status_.phase != RacePhase::kDone) {
      status_.reason = StopReason::kNotReady;
    }
    return false;
  }
  return true;
}

bool Race::search_lost_line(float& yaw_rate) noexcept {
  if (pivot_spent_) {
    return false;
  }
  if (!has_search_heading_) {
    // Bound the sweep by the estimator's own rotation, so the budget means one
    // revolution for any chassis instead of a time constant tuned for this one.
    has_search_heading_ = route_.has_position();
    search_heading_ = has_search_heading_ ? route_.locate_robot().heading : 0.0F;
    if (!has_search_heading_) {
      // Without a heading estimate the sweep cannot be bounded, so it is not
      // started: an unbounded spin is not a search.
      pivot_spent_ = true;
      return false;
    }
  }
  // Both edges lit is a wide patch with no side to prefer, and a full revolution
  // covers either choice.
  yaw_rate = (edge_side_ != 0.0F ? edge_side_ : 1.0F) * kPivotRate;
  const float swept =
    std::abs(std::remainder(route_.locate_robot().heading - search_heading_, kSearchTurns));
  pivot_spent_ = swept > kSearchTurns;
  return true;
}

MotorDemand Race::drive(const SensorReadings& sensors, RaceRequest request) noexcept {
  if (!accept_request(request)) {
    return {};
  }
  update_progress(sensors);
  update_line_state(sensors);

  const bool ir_fresh =
    std::any_of(sensors.ir.begin(), sensors.ir.end(),
                [](const SensorObservation& sample) { return is_fresh(sample, kMaxIrAge); });

  // IR sample age is the health of the steering reference, and nothing about a
  // brick changes it: a hidden line still delivers fresh samples. Once the array
  // has been alive, a stale one means the reference is gone, so braking as hard
  // as the drive allows is the only supported command; crawling blind would keep
  // committing distance on evidence the robot no longer has. Before the first
  // sample the array is starting up, not dead, so the run keeps its unknown-path
  // crawl until the limit decides.
  if (ir_fresh) {
    ir_alive_ = true;
    stale_ir_time_ = 0.0F;
  } else {
    stale_ir_time_ += sensors.elapsed_time;
    if (stale_ir_time_ > kSensorFaultLimit) {
      status_.phase = RacePhase::kFault;
      status_.reason = StopReason::kSensorFault;
      return {};
    }
    if (ir_alive_) {
      has_reference_ = false;
      wheel_reference_.fill(0.0F);
      MotorDemand brake;
      brake.valid_for_time = kCommandValidity;
      brake.mode = MotorMode::kBrake;
      return brake;
    }
  }

  std::array<Region, kRegionCapacity> regions{};
  const std::size_t count = collect_regions(sensors, regions);

  // One continuous field for both duties: the line term is the pure-pursuit arc
  // the tracker would follow anyway, and each relevant region adds a body-right
  // tangential component scaled by the gap to its remembered envelope. There is
  // no switch between "following" and "avoiding" and no stored detour.
  const float last_speed =
    has_reference_ ? std::max(wheel_reference_[0], wheel_reference_[1]) : 0.0F;
  influence_ = geometry_.lookahead + last_speed * (geometry_.motor_response + kMaxWheelAge) +
               last_speed * last_speed / (2.0F * geometry_.brake_deceleration);
  const Point2 line_target = arc_point(line_curvature(), geometry_.lookahead);
  const Point2 target = steer_target(count, regions, line_target);
  const float reach_squared = target.x * target.x + target.y * target.y;
  if (!std::isfinite(reach_squared) || reach_squared < kFieldEpsilon) {
    // The line pull and every relevant region cancel, so no direction is defined.
    status_.phase = RacePhase::kDone;
    status_.reason = StopReason::kNoFeasiblePath;
    return {};
  }
  const float curvature = 2.0F * target.y / reach_squared;

  // Speed auto-scales with curvature via the measured slip table. A missing
  // reading is unknown space, not free space: without any range observation or a
  // usable pose the map cannot vouch for the path, so crawl.
  float ceiling = slip_limit(std::abs(curvature));
  // A differential drive cannot hold an arc whose outer wheel would exceed the
  // top speed: v*(1 + |kappa|/2) <= 1. Clipping the pair afterwards would silently
  // change the curvature the sweep was checked at.
  ceiling = std::min(ceiling, 1.0F / (1.0F + 0.5F * std::abs(curvature)));
  if (geometry_.motor_response > 0.0F) {
    ceiling = std::min(ceiling, kLoopToActuatorRatio * geometry_.lookahead /
                                  (std::sqrt(2.0F) * geometry_.motor_response));
  }
  if (!line_live_) {
    ceiling = std::min(ceiling, kCrawlSpeed);
  }
  if (!ahead_bounded(sensors) || !route_.has_position()) {
    ceiling = std::min(ceiling, kCrawlSpeed);
  }
  const float planned = feasible_speed(ceiling, curvature, regions, count);
  float speed = planned;
  float yaw_rate = 0.0F;
  if (!choose_action(sensors, regions, count, planned, curvature, speed, yaw_rate)) {
    status_.phase = RacePhase::kDone;
    status_.reason = StopReason::kNoFeasiblePath;
    has_reference_ = false;
    return {};
  }

  // A jammed chassis: fresh encoders that show no progress while the robot is
  // commanded forward means it is pressed against a brick. Stop instead of
  // ramming; the safety behaviour is independent of the avoidance field.
  if (jammed(sensors, speed)) {
    status_.phase = RacePhase::kFault;
    status_.reason = StopReason::kBlocked;
    return {};
  }

  const auto wheels = ramp_wheels(speed, yaw_rate, sensors.elapsed_time);
  if (!wheels_clear(wheels, regions, count)) {
    // The planned arc was checked, but the ramp toward it is a different motion.
    // Braking is the only command whose own residual is already inside the sweep.
    has_reference_ = false;
    wheel_reference_.fill(0.0F);
    MotorDemand brake;
    brake.valid_for_time = kCommandValidity;
    brake.mode = MotorMode::kBrake;
    return brake;
  }
  has_reference_ = true;
  return emit_wheels(wheels);
}

}  // namespace lf
