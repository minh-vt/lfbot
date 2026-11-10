#include "route.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lf {
namespace {

constexpr float kPi = 3.14159265358979323846F;
// Normalized sample ages are floats; only absorb their round-off at a boundary.
constexpr double kTimeTolerance = std::numeric_limits<float>::epsilon();

float distance(Point2 a, Point2 b) noexcept {
  return std::hypot(a.x - b.x, a.y - b.y);
}

Point2 place_reading(const RoutePose& pose, Point2 local) noexcept {
  const float c = std::cos(pose.heading);
  const float s = std::sin(pose.heading);
  return {pose.position.x + c * local.x - s * local.y, pose.position.y + s * local.x + c * local.y};
}

void cover_disk(RouteObstacle& obstacle, Point2 centre, float radius) noexcept {
  const float separation = distance(centre, obstacle.centre);
  if (separation + radius <= obstacle.radius) {
    return;
  }
  if (separation + obstacle.radius <= radius) {
    obstacle.centre = centre;
    obstacle.radius = radius;
    return;
  }
  const float expanded = 0.5F * (separation + radius + obstacle.radius);
  const float fraction = (expanded - obstacle.radius) / separation;
  obstacle.centre.x += fraction * (centre.x - obstacle.centre.x);
  obstacle.centre.y += fraction * (centre.y - obstacle.centre.y);
  obstacle.radius = expanded;
}

}  // namespace

Route::Route(const RaceGeometry& geometry, const RobotTuning& tuning) noexcept
  : wheel_resolution_(geometry.wheel_resolution),
    travel_error_(tuning.wheel_travel_error),
    ir_noise_bound_(tuning.ir_noise_bound),
    fit_tolerance_(std::max(geometry.ir_spot_radius, geometry.wheel_resolution)),
    geometry_(geometry) {}

void Route::reset() noexcept {
  run_ = 0;
  line_.size = 0;
  path_.size = 0;
  obstacle_size_ = 0;
  coverage_ = {};
  reset_estimate();
}

void Route::prepare_run() noexcept {
  ++run_;
  reset_estimate();
}

void Route::reset_estimate() noexcept {
  time_ = 0.0;
  first_packet_ = true;
  has_position_ = false;
  pose_ = {};
  motion_ = {};
  wheels_ = {};
  poses_ = {};
  wheel_sequences_.fill(0);
  has_wheel_sequence_.fill(false);
  previous_wheels_.fill(0.0F);
  rejected_samples_ = 0;
  pending_size_ = 0;
  has_ir_sequence_ = false;
  has_range_sequence_ = false;
  last_ir_time_ = 0.0;
  last_range_time_ = 0.0;
  latest_line_ = {};
  line_order_ = 0;
  line_break_ = false;
}

std::optional<double> Route::locate_acquisition(float age) const noexcept {
  if (!std::isfinite(age) || age < 0.0F || time_ - age < -kTimeTolerance) {
    return std::nullopt;
  }
  return std::max(0.0, time_ - age);
}

void Route::record_wheel(std::size_t index, const SensorObservation& sample) noexcept {
  if (sample.status != ReadingStatus::kValid) {
    return;
  }
  if (!std::isfinite(sample.value) || !std::isfinite(sample.age) || sample.age < 0.0F) {
    ++rejected_samples_;
    return;
  }
  if (has_wheel_sequence_[index] && wheel_sequences_[index] == sample.sequence) {
    return;
  }
  double acquisition_time = time_ - sample.age;
  if (acquisition_time < -kTimeTolerance) {
    ++rejected_samples_;
    return;
  }
  acquisition_time = std::max(0.0, acquisition_time);
  auto& history = wheels_[index];
  if (history.size > 0 && acquisition_time <= history.at(history.size - 1U).time + kTimeTolerance) {
    ++rejected_samples_;
    return;
  }
  history.append({acquisition_time, sample.value});
  wheel_sequences_[index] = sample.sequence;
  has_wheel_sequence_[index] = true;
}

std::optional<float> Route::locate_wheel(std::size_t index, double time) const noexcept {
  const auto& history = wheels_[index];
  for (std::size_t i = history.size; i > 0; --i) {
    const auto& sample = history.at(i - 1U);
    if (std::abs(sample.time - time) <= kTimeTolerance) {
      return sample.position;
    }
  }
  return std::nullopt;
}

void Route::update_position(double time, const std::array<float, 2>& wheels) noexcept {
  if (!has_position_) {
    // If the initial encoder origin was never observed, do not pretend that a
    // later zero from Drive proves no motion occurred during the missing span.
    if (time > kTimeTolerance) {
      ++rejected_samples_;
      return;
    }
    previous_wheels_ = wheels;
    pose_.time = time;
    pose_.wheel_position = 0.5F * (wheels[0] + wheels[1]);
    pose_.position_error = 2.0F * wheel_resolution_;
    pose_.heading_error = 4.0F * wheel_resolution_;
    has_position_ = true;
    poses_.append(pose_);
    record_path(0.0F);
    return;
  }
  if (time <= pose_.time + kTimeTolerance) {
    return;
  }
  const double interval = time - pose_.time;
  const float left = wheels[0] - previous_wheels_[0];
  const float right = wheels[1] - previous_wheels_[1];
  const float distance = 0.5F * (left + right);
  const float angle = right - left;  // Wheelbase is one, y points left.
  // For |wheel acceleration| <= A, each velocity differs from its interval
  // mean by at most A*T/2. Its integrated absolute deviation is <= A*T*T/2.
  // This bounds both unobserved reversals and noncommuting wheel profiles.
  const auto variation =
    static_cast<float>(0.5 * geometry_.wheel_acceleration * interval * interval);
  const float profile_error = (std::abs(distance) + variation) * std::min(2.0F, 2.0F * variation) +
                              variation * std::min(2.0F, std::abs(angle));
  if (!std::isfinite(distance) || !std::isfinite(angle) || !std::isfinite(profile_error)) {
    ++rejected_samples_;
    return;
  }
  const float half_angle = 0.5F * angle;
  const float arc_scale = std::abs(half_angle) < 1e-6F ? 1.0F : std::sin(half_angle) / half_angle;
  const float mid = pose_.heading + half_angle;
  const float wheel_motion = 0.5F * (std::abs(left) + std::abs(right));
  const float travel_bound = wheel_motion + variation;
  pose_.position.x += distance * arc_scale * std::cos(mid);
  pose_.position.y += distance * arc_scale * std::sin(mid);
  pose_.heading = std::remainder(pose_.heading + angle, 2.0F * kPi);
  pose_.time = time;
  pose_.wheel_position = 0.5F * (wheels[0] + wheels[1]);
  pose_.wheel_distance += wheel_motion;
  pose_.path_distance += std::abs(distance);
  pose_.travel_bound += travel_bound;
  pose_.profile_error += profile_error;
  // Quantization is an endpoint bound, not fresh independent noise to add on
  // every zero-motion tick. Wheel-slip error remains a conditional budget.
  pose_.heading_error = static_cast<float>(
    std::min<double>(kPi, 2.0 * travel_error_ * pose_.travel_bound + 4.0 * wheel_resolution_));
  pose_.position_error +=
    travel_bound * (travel_error_ + std::min(2.0F, pose_.heading_error)) + profile_error;
  previous_wheels_ = wheels;
  motion_ = {distance, angle};
  poses_.append(pose_);
  record_path(std::abs(distance) + variation + travel_error_ * travel_bound);
}

void Route::record_readings(const SensorReadings& sensors) noexcept {
  if (run_ == 0) {
    run_ = 1;
  }
  motion_ = {};
  if (!std::isfinite(sensors.elapsed_time) || sensors.elapsed_time < 0.0F) {
    ++rejected_samples_;
    return;
  }
  if (first_packet_) {
    first_packet_ = false;
  } else {
    time_ += sensors.elapsed_time;
  }
  for (std::size_t i = 0; i < wheels_.size(); ++i) {
    record_wheel(i, sensors.wheel_positions[i]);
  }
  if (wheels_[0].size > 0 && wheels_[1].size > 0) {
    const double common_time =
      std::min(wheels_[0].at(wheels_[0].size - 1U).time, wheels_[1].at(wheels_[1].size - 1U).time);
    const auto left = locate_wheel(0, common_time);
    const auto right = locate_wheel(1, common_time);
    if (left.has_value() && right.has_value()) {
      update_position(common_time, {*left, *right});
    }
  }
  queue_line(sensors);
  queue_range(sensors.tof[0]);
  process_readings();
}

std::optional<RoutePose> Route::locate_robot(double acquisition_time) const noexcept {
  if (!std::isfinite(acquisition_time)) {
    return std::nullopt;
  }
  // Only acquired epochs are supported. Constant wheel velocity/curvature
  // inside an unobserved interval would be an extra model, not a measurement.
  for (std::size_t i = poses_.size; i > 0; --i) {
    const auto& sample = poses_.at(i - 1U);
    if (std::abs(sample.time - acquisition_time) <= kTimeTolerance) {
      return sample;
    }
  }
  return std::nullopt;
}

void Route::queue_reading(const PendingReading& sample) noexcept {
  if (pending_size_ == kPendingCapacity) {
    coverage_.overflow = true;
    return;
  }
  pending_[pending_size_++] = sample;
}

void Route::queue_line(const SensorReadings& sensors) noexcept {
  bool changed = !has_ir_sequence_;
  const float age = sensors.ir[0].age;
  PendingReading reading;
  for (std::size_t i = 0; i < sensors.ir.size(); ++i) {
    const auto& sample = sensors.ir[i];
    if (sample.status != ReadingStatus::kValid) {
      return;
    }
    if (!std::isfinite(sample.value) || sample.value < 0.0F || sample.value > 1.0F ||
        !std::isfinite(sample.age) || std::abs(sample.age - age) > kTimeTolerance) {
      ++rejected_samples_;
      mark_line_gap(0.0, time_);
      return;
    }
    changed = changed || sample.sequence != ir_sequences_[i];
    reading.values[i] = sample.value;
  }
  if (!changed) {
    return;
  }
  const auto acquisition = locate_acquisition(age);
  if (!acquisition.has_value() ||
      (has_ir_sequence_ && *acquisition <= last_ir_time_ + kTimeTolerance)) {
    ++rejected_samples_;
    mark_line_gap(0.0, time_);
    return;
  }
  for (std::size_t i = 0; i < sensors.ir.size(); ++i) {
    ir_sequences_[i] = sensors.ir[i].sequence;
  }
  reading.break_before =
    has_ir_sequence_ && (geometry_.ir_interval <= 0.0F ||
                         *acquisition - last_ir_time_ > geometry_.ir_interval + kTimeTolerance);
  has_ir_sequence_ = true;
  last_ir_time_ = *acquisition;
  reading.time = *acquisition;
  queue_reading(reading);
}

void Route::mark_line_gap(double first, double last) noexcept {
  // Keep the whole interval of rejected epochs until a later valid acquisition
  // crosses it. A delayed, older observation must not consume a newer break.
  if (line_break_) {
    first_line_break_time_ = std::min(first_line_break_time_, first);
    last_line_break_time_ = std::max(last_line_break_time_, last);
  } else {
    first_line_break_time_ = first;
    last_line_break_time_ = last;
  }
  line_break_ = true;
}

void Route::queue_range(const SensorObservation& sample) noexcept {
  if (sample.status != ReadingStatus::kValid && sample.status != ReadingStatus::kNoReturn) {
    return;
  }
  if (has_range_sequence_ && sample.sequence == range_sequence_) {
    return;
  }
  const auto acquisition = locate_acquisition(sample.age);
  if (!acquisition.has_value() || !std::isfinite(sample.value) || sample.value < 0.0F ||
      (has_range_sequence_ && *acquisition <= last_range_time_ + kTimeTolerance)) {
    ++rejected_samples_;
    return;
  }
  has_range_sequence_ = true;
  range_sequence_ = sample.sequence;
  last_range_time_ = *acquisition;
  PendingReading reading;
  reading.time = *acquisition;
  reading.range = true;
  reading.status = sample.status;
  reading.values[0] = sample.value;
  queue_reading(reading);
}

void Route::process_readings() noexcept {
  std::size_t remaining = 0;
  // At most kPendingCapacity lookups, each at most kHistoryCapacity entries.
  for (std::size_t i = 0; i < pending_size_; ++i) {
    const auto& sample = pending_[i];
    const auto acquisition_pose = locate_robot(sample.time);
    if (acquisition_pose.has_value()) {
      if (sample.range) {
        ++coverage_.range_acquisitions;
        record_obstacle(sample, *acquisition_pose);
      } else {
        ++coverage_.line_acquisitions;
        record_line(sample, *acquisition_pose);
      }
    } else if (!has_position_ || sample.time > pose_.time + kTimeTolerance) {
      pending_[remaining++] = sample;
    } else {
      ++rejected_samples_;
      coverage_.has_gap = true;
      if (!sample.range) {
        mark_line_gap(sample.time, sample.time);
      }
    }
  }
  pending_size_ = remaining;
}

bool Route::record_point(Curve& curve, RoutePoint point) const noexcept {
  if (curve.size == 1 && !point.break_before && point.evidence == curve.points[0].evidence &&
      distance(point.position, curve.points[0].position) <= fit_tolerance_) {
    curve.points[0].error = std::max(curve.points[0].error, point.error);
    curve.points[0].fit_error = fit_tolerance_;
    return true;
  }
  if (curve.size >= 2) {
    const auto& start = curve.points[curve.size - 2U];
    auto& end = curve.points[curve.size - 1U];
    const float radius = distance(point.position, start.position);
    const float angle = std::remainder(
      std::atan2(point.position.y - start.position.y, point.position.x - start.position.x) -
        curve.direction,
      2.0F * kPi);
    if (!point.break_before && !end.break_before && start.evidence == point.evidence &&
        end.evidence == point.evidence && radius >= curve.max_radius && angle >= curve.min_angle &&
        angle <= curve.max_angle) {
      if (radius > fit_tolerance_) {
        const float spread = std::asin(std::clamp(fit_tolerance_ / radius, 0.0F, 1.0F));
        curve.min_angle = std::max(curve.min_angle, angle - spread);
        curve.max_angle = std::min(curve.max_angle, angle + spread);
      }
      curve.max_radius = radius;
      point.error = std::max(point.error, end.error);
      point.fit_error = fit_tolerance_;
      end = point;
      return true;
    }
  }
  if (curve.size == kCurveCapacity) {
    return false;
  }
  if (curve.size > 0) {
    const auto& previous = curve.points[curve.size - 1U];
    const float radius = distance(point.position, previous.position);
    curve.direction =
      std::atan2(point.position.y - previous.position.y, point.position.x - previous.position.x);
    const float spread =
      radius > fit_tolerance_ ? std::asin(std::clamp(fit_tolerance_ / radius, 0.0F, 1.0F)) : kPi;
    curve.min_angle = -spread;
    curve.max_angle = spread;
    curve.max_radius = radius;
    point.fit_error = fit_tolerance_;
  }
  curve.points[curve.size++] = point;
  return true;
}

void Route::record_path(float interval_travel) noexcept {
  if (run_ != 1 || coverage_.overflow) {
    return;
  }
  RoutePoint point;
  point.position = pose_.position;
  point.path_distance = pose_.path_distance;
  // Include the unretained nominal arc between encoder epochs, not only its endpoints.
  point.error = pose_.position_error + interval_travel;
  point.evidence = RouteEvidence::kTracedPath;
  if (!record_point(path_, point)) {
    coverage_.overflow = true;
  }
  coverage_.recorded_distance = path_.points[path_.size - 1U].path_distance;
}

void Route::record_line(const PendingReading& sample, const RoutePose& pose) noexcept {
  Point2 local;
  float weight = 0.0F;
  std::size_t lit = 0;
  std::size_t clusters = 0;
  bool previous = false;
  for (std::size_t i = 0; i < sample.values.size(); ++i) {
    const bool black = sample.values[i] > ir_noise_bound_;
    if (black) {
      clusters += previous ? 0U : 1U;
      ++lit;
      weight += sample.values[i];
      local.x += sample.values[i] * geometry_.ir_position[i].x;
      local.y += sample.values[i] * geometry_.ir_position[i].y;
    }
    previous = black;
  }
  RoutePoint point;
  point.path_distance = pose.path_distance;
  point.order = ++line_order_;
  const bool rejected_before =
    line_break_ && sample.time + kTimeTolerance >= first_line_break_time_;
  point.break_before = sample.break_before || rejected_before;
  if (rejected_before && sample.time > last_line_break_time_ + kTimeTolerance) {
    line_break_ = false;
  }
  point.evidence = RouteEvidence::kGap;
  if (weight > 0.0F) {
    local.x /= weight;
    local.y /= weight;
    point.evidence =
      clusters == 1 && lit < kIrSensorCount ? RouteEvidence::kLine : RouteEvidence::kAmbiguous;
  } else {
    // The locus of an unsupported patch, not an invented line measurement.
    local = {geometry_.lookahead, 0.0F};
  }
  float spread = 0.0F;
  for (std::size_t i = 0; i < sample.values.size(); ++i) {
    if (sample.values[i] > ir_noise_bound_) {
      spread = std::max(spread, distance(local, geometry_.ir_position[i]));
    }
  }
  point.position = place_reading(pose, local);
  point.error = pose.position_error +
                std::hypot(local.x, local.y) * std::min(2.0F, pose.heading_error) + spread +
                geometry_.line_half_width + geometry_.ir_spot_radius;
  latest_line_ = point;
  coverage_.has_gap =
    coverage_.has_gap || point.break_before || point.evidence == RouteEvidence::kGap;
  coverage_.has_ambiguity = coverage_.has_ambiguity || point.evidence == RouteEvidence::kAmbiguous;
  if (run_ == 1 && !coverage_.overflow && !record_point(line_, point)) {
    coverage_.overflow = true;
  }
}

float Route::transport_error(const RouteObstacle& obstacle, const RoutePose& pose,
                             float reach) const noexcept {
  if (obstacle.run == run_ && std::abs(pose.time - obstacle.anchor.time) <= kTimeTolerance) {
    return 0.0F;
  }
  const float separation = distance(pose.position, obstacle.anchor.position);
  const float lever =
    std::max(distance(obstacle.centre, obstacle.anchor.position), reach + separation);
  if (obstacle.run != run_) {
    // Worst-case bounds, not independent covariance fusion across runs.
    return obstacle.anchor.position_error + lever * std::min(2.0F, obstacle.anchor.heading_error) +
           pose.position_error + reach * std::min(2.0F, pose.heading_error);
  }
  const auto motion =
    static_cast<float>(std::abs(pose.travel_bound - obstacle.anchor.travel_bound));
  const auto profile_error =
    static_cast<float>(std::abs(pose.profile_error - obstacle.anchor.profile_error));
  const float heading_error =
    std::min(2.0F, 2.0F * travel_error_ * motion + 4.0F * wheel_resolution_);
  return travel_error_ * motion + motion * heading_error + 2.0F * wheel_resolution_ +
         lever * heading_error + profile_error;
}

void Route::record_obstacle(const PendingReading& sample, const RoutePose& pose) noexcept {
  if (sample.status == ReadingStatus::kNoReturn || coverage_.overflow) {
    return;
  }
  const float range = sample.values[0];
  const Point2 centre = place_reading(pose, {geometry_.tof_forward + range, 0.0F});
  // The ToF reports a bearing and a range, so what it measured is a patch on the
  // surface: half the cone width at that range, plus the range error. Growing
  // that patch by the object's own diameter would put a 205 mm brick's far side
  // inside a disk centred on its near face, which forbids every approach to the
  // brick it came from. How far the body continues behind the patch is not
  // observed; the planner must not read the unobserved side as free space.
  const float radius = observed_patch_radius(geometry_, range);
  for (std::size_t i = 0; i < obstacle_size_; ++i) {
    auto& obstacle = obstacles_[i];
    const float in_anchor = radius + transport_error(obstacle, pose, geometry_.tof_forward + range);
    if (distance(centre, obstacle.centre) <= in_anchor + obstacle.radius) {
      // Association is not identifiable from scalar ranges: preserve the union
      // even if these are two objects. Never intersect them to fake certainty.
      // The union is over observed envelopes only: pose-gauge error says where
      // the record could be, not how large the object is, so folding it into the
      // shape would grow a far-away brick until it covers the whole track.
      cover_disk(obstacle, centre, radius);
      return;
    }
  }
  if (obstacle_size_ == kObstacleCapacity) {
    coverage_.overflow = true;
    return;
  }
  obstacles_[obstacle_size_++] = {centre, radius, 0.0F, pose, run_};
}

RouteObstacle Route::locate_obstacle(std::size_t i, const RoutePose& pose) const noexcept {
  auto result = obstacles_[i];
  result.uncertainty = transport_error(result, pose, distance(result.centre, pose.position));
  return result;
}

RouteCorrespondence Route::locate_line(const RoutePose& pose) const noexcept {
  RouteCorrespondence result;
  std::size_t previous_segment = 0;
  for (std::size_t i = 1; i < line_.size; ++i) {
    const auto& a = line_.points[i - 1U];
    const auto& b = line_.points[i];
    if (b.break_before || a.evidence != RouteEvidence::kLine ||
        b.evidence != RouteEvidence::kLine) {
      continue;
    }
    const float dx = b.position.x - a.position.x;
    const float dy = b.position.y - a.position.y;
    const float squared = dx * dx + dy * dy;
    const float fraction = squared > 0.0F ? std::clamp(((pose.position.x - a.position.x) * dx +
                                                        (pose.position.y - a.position.y) * dy) /
                                                         squared,
                                                       0.0F, 1.0F)
                                          : 0.0F;
    const Point2 projection{a.position.x + fraction * dx, a.position.y + fraction * dy};
    const float error =
      std::max(a.error + a.fit_error, b.error + b.fit_error) + pose.position_error;
    if (distance(pose.position, projection) > error) {
      continue;
    }
    if (!result.supported) {
      result.begin_order = static_cast<float>(a.order);
      result.begin_distance = a.path_distance;
    } else {
      result.ambiguous = result.ambiguous || i > previous_segment + 1U;
    }
    result.supported = true;
    result.end_order = static_cast<float>(b.order);
    result.end_distance = b.path_distance;
    previous_segment = i;
  }
  return result;
}

}  // namespace lf
