#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <robot_data.hpp>

#include "robot_geometry.hpp"
#include "track_asset.hpp"

namespace lf::sim {

// World/asset coordinates are raster x-right, y-down, yaw clockwise.
// Robot-local coordinates remain x-forward, y-left.
struct Pose {
  float x_mm = 0.0F;
  float y_mm = 0.0F;
  float yaw_rad = 0.0F;
  float speed_mm_s = 0.0F;
};

inline constexpr std::size_t kSensorQueueCapacity = 16;

struct SensorTiming {
  // Acquisition/delivery occur on the observation clock, without backfilling
  // skipped observations. Engine defaults are multiples of its 2 ms tick.
  std::uint32_t period_us = 2000;
  std::uint32_t delay_us = 0;
  // Acquisition outage relative to the first observation after reset. Earlier
  // queued samples can still arrive; absent new samples, the last one ages.
  std::uint64_t dropout_begin_us = 0;
  std::uint64_t dropout_end_us = 0;
};

// Synthetic plant parameters, not measured hardware calibration or controller
// gains. Kept independent of the robot-known reference scale/bounds.
struct WorldParameters {
  float max_wheel_speed_mm_s = 500.0F;
  float drive_response_s = 0.060F;
  float drive_acceleration_mm_s2 = 2000.0F;
  float brake_deceleration_mm_s2 = 4000.0F;
  float coast_deceleration_mm_s2 = 250.0F;
  std::array<float, 2> traction = {1.0F, 1.0F};
  SensorTiming ir;
  SensorTiming tof{20000};
  SensorTiming encoders;
  float ir_noise_fraction = 0.0F;
  float tof_noise_mm = 0.0F;
  std::uint32_t noise_seed = 17;
};

[[nodiscard]] bool validate_world_parameters(const WorldParameters& parameters) noexcept;

class WorldPhysics {
public:
  explicit WorldPhysics(const Calibration& calibration = simulated_calibration(),
                        const WorldParameters& parameters = {}) noexcept;
  void reset(const TrackAsset& track) noexcept;
  [[nodiscard]] SensorMeasurements sense(std::uint64_t time_us, const TrackAsset& track) noexcept;
  void advance(const MotorOutput& output, std::uint64_t begin_us, std::uint64_t end_us,
               const TrackAsset& track) noexcept;
  [[nodiscard]] const Pose& pose() const noexcept { return pose_; }
  [[nodiscard]] bool blocked() const noexcept { return blocked_; }
  [[nodiscard]] bool parameters_valid() const noexcept { return parameters_valid_; }
  [[nodiscard]] bool collided(const TrackAsset& track) const noexcept;

private:
  template <typename T>
  struct SensorChannel {
    static constexpr std::size_t kCapacity = kSensorQueueCapacity;
    std::array<SensorSample<T>, kCapacity> pending{};
    SensorSample<T> delivered;
    std::size_t head = 0;
    std::size_t size = 0;
    std::uint64_t last_acquisition_us = 0;
    std::uint32_t sequence = 0;
    bool acquired = false;

    [[nodiscard]] bool due(std::uint64_t now_us, std::uint64_t origin_us, std::uint32_t period_us,
                           bool immediate) const noexcept {
      if (!acquired) {
        return immediate || now_us - origin_us >= period_us;
      }
      return now_us - last_acquisition_us >= period_us;
    }
    void publish(std::uint64_t now_us, std::uint32_t delay_us) noexcept {
      while (size > 0 && now_us - pending[head].time_us >= delay_us) {
        delivered = pending[head];
        head = (head + 1U) % kCapacity;
        --size;
      }
    }
    void acquire(T value, std::uint64_t now_us, ReadingStatus status) noexcept {
      // Config validation bounds delay/period; publish precedes acquisition,
      // so at most kCapacity-1 entries can be pending here.
      pending[(head + size) % kCapacity] = {value, now_us, ++sequence, status};
      ++size;
      last_acquisition_us = now_us;
      acquired = true;
    }
  };

  struct Range {
    float distance_mm;
    ReadingStatus status;
  };
  [[nodiscard]] Range front_range(const TrackAsset& track) const noexcept;
  [[nodiscard]] float sample_noise() noexcept;
  void advance_step(const MotorOutput& output, float elapsed_s, const TrackAsset& track) noexcept;

  Calibration calibration_;
  WorldParameters parameters_;
  bool parameters_valid_;
  Pose pose_;
  std::array<float, 2> wheel_speed_mm_s_{};
  std::array<SensorChannel<float>, kIrSensorCount> ir_channels_;
  SensorChannel<float> tof_channel_;
  std::array<SensorChannel<std::uint32_t>, 2> encoder_channels_;
  double wheel_left_mm_ = 0.0;
  double wheel_right_mm_ = 0.0;
  std::uint64_t sensor_origin_us_ = 0;
  std::uint64_t last_sense_us_ = 0;
  bool has_sensor_time_ = false;
  std::uint32_t noise_state_ = 17;
  bool blocked_ = false;
};

}  // namespace lf::sim
