#include "world_physics.hpp"

#include <algorithm>
#include <cmath>

#include "robot_geometry.hpp"

namespace lf::sim {
namespace {

constexpr float kPi = 3.14159265358979323846F;
constexpr std::uint64_t kPhysicsStepUs = 250;

// A real IR channel integrates an analog spot, it does not read one pixel.
float sample_spot(const TrackAsset& track, float x_mm, float y_mm, float radius_mm) noexcept {
  constexpr int kSteps = 2;  // 5x5 grid clamped to the spot disc
  const float step = radius_mm / static_cast<float>(kSteps);
  float sum = 0.0F;
  float weight = 0.0F;
  for (int row = -kSteps; row <= kSteps; ++row) {
    for (int column = -kSteps; column <= kSteps; ++column) {
      const float dx = static_cast<float>(column) * step;
      const float dy = static_cast<float>(row) * step;
      if (dx * dx + dy * dy > radius_mm * radius_mm) {
        continue;
      }
      sum += static_cast<float>(track.sample(x_mm + dx, y_mm + dy));
      weight += 1.0F;
    }
  }
  return weight > 0.0F ? sum / weight : static_cast<float>(track.sample(x_mm, y_mm));
}

float wrap(float angle) noexcept {
  return std::remainder(angle, 2.0F * kPi);
}

bool ray_box(float ox, float oy, float dx, float dy, const Obstacle& box, float max_range,
             float& distance) noexcept {
  const float cosine = std::cos(box.angle_rad);
  const float sine = std::sin(box.angle_rad);
  const float local_x = (ox - box.x_mm) * cosine + (oy - box.y_mm) * sine;
  const float local_y = -(ox - box.x_mm) * sine + (oy - box.y_mm) * cosine;
  const float local_dx = dx * cosine + dy * sine;
  const float local_dy = -dx * sine + dy * cosine;
  float lower = 0.0F;
  float upper = max_range;
  for (int axis = 0; axis < 2; ++axis) {
    const float origin = axis == 0 ? local_x : local_y;
    const float direction = axis == 0 ? local_dx : local_dy;
    const float half_extent = axis == 0 ? box.width_mm * 0.5F : box.height_mm * 0.5F;
    if (std::abs(direction) < 1e-7F) {
      if (origin < -half_extent || origin > half_extent) {
        return false;
      }
      continue;
    }
    float first = (-half_extent - origin) / direction;
    float last = (half_extent - origin) / direction;
    if (first > last) {
      std::swap(first, last);
    }
    lower = std::max(lower, first);
    upper = std::min(upper, last);
  }
  if (lower > upper || upper < 0.0F || lower > max_range) {
    return false;
  }
  distance = std::max(0.0F, lower);
  return true;
}

bool accepts_timing(const SensorTiming& timing) noexcept {
  return timing.period_us > 0 && timing.delay_us / timing.period_us < kSensorQueueCapacity &&
         timing.dropout_end_us >= timing.dropout_begin_us;
}

bool accepts_resolution(const Calibration& calibration,
                        const WorldParameters& parameters) noexcept {
  const double speed = parameters.max_wheel_speed_mm_s;
  const double duration = static_cast<double>(kPhysicsStepUs) * 1e-6;
  const double radius = std::hypot(static_cast<double>(calibration.half_length_mm),
                                   static_cast<double>(calibration.half_width_mm));
  // Reject a plant whose maximum point motion can cross the robot's own
  // half-extent in one numerical step, or overflow a run's signed tick delta.
  const double point_motion = speed * duration * (1.0 + 2.0 * radius / calibration.wheelbase_mm);
  return point_motion < std::min(calibration.half_length_mm, calibration.half_width_mm) &&
         speed * 180.0 / calibration.wheel_mm_per_tick <= 2147483647.0;
}

bool in_dropout(const SensorTiming& timing, std::uint64_t relative_us) noexcept {
  return relative_us >= timing.dropout_begin_us && relative_us < timing.dropout_end_us;
}

SensorMeasurements invalid_measurements(std::uint64_t time_us) noexcept {
  SensorMeasurements readings;
  readings.time_us = time_us;
  for (auto& sample : readings.ir) {
    sample.status = ReadingStatus::kInvalid;
  }
  for (auto& sample : readings.tof) {
    sample.status = ReadingStatus::kInvalid;
  }
  for (auto& sample : readings.wheel_counters) {
    sample.status = ReadingStatus::kInvalid;
  }
  return readings;
}

float advance_wheel(float speed, float duty, MotorMode mode, float elapsed_s,
                    const WorldParameters& parameters) noexcept {
  if (mode == MotorMode::kDrive) {
    const float target = duty * parameters.max_wheel_speed_mm_s;
    const float response = (target - speed) * -std::expm1(-elapsed_s / parameters.drive_response_s);
    const float limit = parameters.drive_acceleration_mm_s2 * elapsed_s;
    return speed + std::clamp(response, -limit, limit);
  }
  const float deceleration = mode == MotorMode::kBrake ? parameters.brake_deceleration_mm_s2
                                                       : parameters.coast_deceleration_mm_s2;
  const float limit = deceleration * elapsed_s;
  return speed - std::clamp(speed, -limit, limit);
}

}  // namespace

bool validate_world_parameters(const WorldParameters& parameters) noexcept {
  const std::array<float, 5> positive = {
    parameters.max_wheel_speed_mm_s,     parameters.drive_response_s,
    parameters.drive_acceleration_mm_s2, parameters.brake_deceleration_mm_s2,
    parameters.coast_deceleration_mm_s2,
  };
  for (const float value : positive) {
    if (!std::isfinite(value) || value <= 0.0F) {
      return false;
    }
  }
  for (const float value : parameters.traction) {
    if (!std::isfinite(value) || value < 0.0F || value > 1.0F) {
      return false;
    }
  }
  return accepts_timing(parameters.ir) && accepts_timing(parameters.tof) &&
         accepts_timing(parameters.encoders) && std::isfinite(parameters.ir_noise_fraction) &&
         parameters.ir_noise_fraction >= 0.0F && parameters.ir_noise_fraction <= 1.0F &&
         std::isfinite(parameters.tof_noise_mm) && parameters.tof_noise_mm >= 0.0F;
}

WorldPhysics::WorldPhysics(const Calibration& calibration,
                           const WorldParameters& parameters) noexcept
  : calibration_(calibration),
    parameters_(parameters),
    parameters_valid_(validate_calibration(calibration) && validate_world_parameters(parameters) &&
                      accepts_resolution(calibration, parameters)) {}

void WorldPhysics::reset(const TrackAsset& track) noexcept {
  pose_ = {track.start_x_mm, track.start_y_mm, track.start_yaw_rad, 0.0F};
  ir_channels_ = {};
  tof_channel_ = {};
  encoder_channels_ = {};
  sensor_origin_us_ = 0;
  last_sense_us_ = 0;
  has_sensor_time_ = false;
  noise_state_ = parameters_.noise_seed;
  wheel_left_mm_ = 0.0;
  wheel_right_mm_ = 0.0;
  wheel_speed_mm_s_.fill(0.0F);
  blocked_ = false;
}

bool WorldPhysics::collided(const TrackAsset& track) const noexcept {
  const float half_length = calibration_.half_length_mm;
  const float half_width = calibration_.half_width_mm;
  const float robot_c = std::cos(pose_.yaw_rad);
  const float robot_s = std::sin(pose_.yaw_rad);
  for (const auto& box : track.obstacles) {
    const float box_c = std::cos(box.angle_rad);
    const float box_s = std::sin(box.angle_rad);
    const float dx = pose_.x_mm - box.x_mm;
    const float dy = pose_.y_mm - box.y_mm;
    // Separating-axis test: both face normals of both rectangles. Testing only
    // one normal per box misses separation along the remaining axes.
    const std::array<float, 8> axes = {
      robot_c, robot_s, -robot_s, robot_c, box_c, box_s, -box_s, box_c,
    };
    bool separated = false;
    for (std::size_t i = 0; i < axes.size(); i += 2) {
      const float ax = axes[i];
      const float ay = axes[i + 1];
      const float robot_radius = half_length * std::abs(ax * robot_c + ay * robot_s) +
                                 half_width * std::abs(-ax * robot_s + ay * robot_c);
      const float box_radius = box.width_mm * 0.5F * std::abs(ax * box_c + ay * box_s) +
                               box.height_mm * 0.5F * std::abs(-ax * box_s + ay * box_c);
      if (std::abs(dx * ax + dy * ay) > robot_radius + box_radius) {
        separated = true;
        break;
      }
    }
    if (!separated) {
      return true;
    }
  }
  return false;
}

WorldPhysics::Range WorldPhysics::front_range(const TrackAsset& track) const noexcept {
  const float ox = pose_.x_mm + calibration_.tof_forward_mm * std::cos(pose_.yaw_rad);
  const float oy = pose_.y_mm + calibration_.tof_forward_mm * std::sin(pose_.yaw_rad);
  const float half_fov = calibration_.tof_half_fov_rad;
  Range nearest{calibration_.tof_range_mm, ReadingStatus::kNoReturn};
  // Nine rays approximate the FOV; no return does not certify the cone is free.
  constexpr int kRays = 9;
  for (int ray = 0; ray < kRays; ++ray) {
    const float ratio = static_cast<float>(ray) / static_cast<float>(kRays - 1);
    const float angle = pose_.yaw_rad - half_fov + 2.0F * half_fov * ratio;
    for (const auto& obstacle : track.obstacles) {
      float range = 0.0F;
      if (ray_box(ox, oy, std::cos(angle), std::sin(angle), obstacle, calibration_.tof_range_mm,
                  range) &&
          range <= nearest.distance_mm) {
        nearest = {range, ReadingStatus::kValid};
      }
    }
  }
  return nearest;
}

float WorldPhysics::sample_noise() noexcept {
  // Fixed PRNG for reproducible fixtures, advanced at acquisition only.
  noise_state_ = noise_state_ * 1664525U + 1013904223U;
  return 2.0F * static_cast<float>(noise_state_ >> 8U) / 16777215.0F - 1.0F;
}

SensorMeasurements WorldPhysics::sense(std::uint64_t time_us, const TrackAsset& track) noexcept {
  if (!parameters_valid_ || (has_sensor_time_ && time_us < last_sense_us_)) {
    return invalid_measurements(time_us);
  }
  if (!has_sensor_time_) {
    sensor_origin_us_ = time_us;
    has_sensor_time_ = true;
  }
  last_sense_us_ = time_us;
  const auto relative_us = time_us - sensor_origin_us_;
  SensorMeasurements measurements;
  measurements.time_us = time_us;
  for (std::size_t i = 0; i < kIrSensorCount; ++i) {
    auto& channel = ir_channels_[i];
    channel.publish(time_us, parameters_.ir.delay_us);
    if (!in_dropout(parameters_.ir, relative_us) &&
        channel.due(time_us, sensor_origin_us_, parameters_.ir.period_us, true)) {
      const auto& local = calibration_.ir_position_mm[i];
      const float sensor_x =
        pose_.x_mm + local.x * std::cos(pose_.yaw_rad) + local.y * std::sin(pose_.yaw_rad);
      const float sensor_y =
        pose_.y_mm + local.x * std::sin(pose_.yaw_rad) - local.y * std::cos(pose_.yaw_rad);
      const float white_fraction =
        sample_spot(track, sensor_x, sensor_y, calibration_.ir_spot_radius_mm) / 255.0F;
      const float noise = parameters_.ir_noise_fraction * sample_noise();
      const float fraction = std::clamp(white_fraction + noise, 0.0F, 1.0F);
      const float value =
        calibration_.ir_black[i] + fraction * (calibration_.ir_white[i] - calibration_.ir_black[i]);
      channel.acquire(value, time_us, ReadingStatus::kValid);
    }
    channel.publish(time_us, parameters_.ir.delay_us);
    measurements.ir[i] = channel.delivered;
  }
  for (std::size_t i = 0; i < encoder_channels_.size(); ++i) {
    auto& channel = encoder_channels_[i];
    channel.publish(time_us, parameters_.encoders.delay_us);
    if (!in_dropout(parameters_.encoders, relative_us) &&
        channel.due(time_us, sensor_origin_us_, parameters_.encoders.period_us, true)) {
      const double travel = i == 0 ? wheel_left_mm_ : wheel_right_mm_;
      const auto counter = static_cast<std::uint32_t>(
        static_cast<std::int64_t>(travel / calibration_.wheel_mm_per_tick));
      channel.acquire(counter, time_us, ReadingStatus::kValid);
    }
    channel.publish(time_us, parameters_.encoders.delay_us);
    measurements.wheel_counters[i] = channel.delivered;
  }
  tof_channel_.publish(time_us, parameters_.tof.delay_us);
  if (!in_dropout(parameters_.tof, relative_us) &&
      tof_channel_.due(time_us, sensor_origin_us_, parameters_.tof.period_us, false)) {
    const auto range = front_range(track);
    float value = range.distance_mm;
    if (range.status == ReadingStatus::kValid) {
      value = std::clamp(value + parameters_.tof_noise_mm * sample_noise(), 0.0F,
                         calibration_.tof_range_mm);
    }
    tof_channel_.acquire(value, time_us, range.status);
  }
  tof_channel_.publish(time_us, parameters_.tof.delay_us);
  measurements.tof[0] = tof_channel_.delivered;
  return measurements;
}

void WorldPhysics::advance_step(const MotorOutput& output, float elapsed_s,
                                const TrackAsset& track) noexcept {
  const auto previous_speed = wheel_speed_mm_s_;
  wheel_speed_mm_s_[0] =
    advance_wheel(previous_speed[0], output.left_duty, output.mode, elapsed_s, parameters_);
  wheel_speed_mm_s_[1] =
    advance_wheel(previous_speed[1], output.right_duty, output.mode, elapsed_s, parameters_);
  const float left = 0.5F * (previous_speed[0] + wheel_speed_mm_s_[0]) * elapsed_s;
  const float right = 0.5F * (previous_speed[1] + wheel_speed_mm_s_[1]) * elapsed_s;
  // Encoders measure axle rotation, even when contact or slip prevents travel.
  wheel_left_mm_ += left;
  wheel_right_mm_ += right;
  const float body_left = left * parameters_.traction[0];
  const float body_right = right * parameters_.traction[1];
  const float distance = 0.5F * (body_left + body_right);
  // Reflection from robot y-left to raster y-down reverses yaw handedness.
  const float angle = (body_left - body_right) / calibration_.wheelbase_mm;
  const float half_angle = 0.5F * angle;
  const float arc_scale = std::abs(half_angle) < 1e-6F ? 1.0F : std::sin(half_angle) / half_angle;
  const Pose previous = pose_;
  const float mid = previous.yaw_rad + half_angle;
  pose_.x_mm += distance * arc_scale * std::cos(mid);
  pose_.y_mm += distance * arc_scale * std::sin(mid);
  pose_.yaw_rad = wrap(previous.yaw_rad + angle);
  pose_.speed_mm_s = 0.5F * (wheel_speed_mm_s_[0] * parameters_.traction[0] +
                             wheel_speed_mm_s_[1] * parameters_.traction[1]);
  if (collided(track)) {
    // Inelastic, no-slide contact fixture: reject the complete body step, not
    // just translation. Motor/axle state is independent and can keep spinning.
    pose_ = previous;
    pose_.speed_mm_s = 0.0F;
    blocked_ = true;
  }
}

void WorldPhysics::advance(const MotorOutput& output, std::uint64_t begin_us, std::uint64_t end_us,
                           const TrackAsset& track) noexcept {
  blocked_ = false;
  if (!parameters_valid_ || end_us <= begin_us) {
    return;
  }
  const bool valid = std::isfinite(output.left_duty) && std::isfinite(output.right_duty) &&
                     std::abs(output.left_duty) <= 1.0F && std::abs(output.right_duty) <= 1.0F &&
                     (output.mode == MotorMode::kDrive || output.mode == MotorMode::kBrake ||
                      output.mode == MotorMode::kCoast);
  // Numerical resolution, not a controller gain; eight steps per engine tick.
  for (auto now_us = begin_us; now_us < end_us;) {
    auto duration_us = std::min(kPhysicsStepUs, end_us - now_us);
    const bool active = valid && now_us < output.deadline_us;
    if (active) {
      duration_us = std::min(duration_us, output.deadline_us - now_us);
    }
    const MotorOutput command = active ? output : MotorOutput{};
    advance_step(command, static_cast<float>(duration_us) * 1e-6F, track);
    now_us += duration_us;
  }
}

}  // namespace lf::sim
