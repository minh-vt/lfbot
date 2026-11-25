#pragma once

#include <array>
#include <cstdint>

#include "config.hpp"
#include "types.hpp"

namespace lf {

// What the robot is doing, from the wheels and the gyro: speed, distance
// travelled, heading and where it is on the floor. Forward is positive; positive
// heading is a left turn; x is straight ahead and y to the left of the pose at zero().
class Motion {
public:
  void update(const Measurements& measurements);
  // Distance and heading restart from zero here.
  void zero();

  // Gyro bias is learned from a block of samples taken while the robot stands
  // still. begin_bias starts the search; update_bias returns true once a still
  // block has been found, and keeps trying with fresh blocks until then.
  void begin_bias();
  [[nodiscard]] bool update_bias(std::uint64_t now_us);

  [[nodiscard]] float wheel_cps(std::size_t wheel) const { return wheel_cps_[wheel]; }
  [[nodiscard]] float s_counts() const;
  [[nodiscard]] float psi_rad() const { return psi_rad_; }
  [[nodiscard]] float x_mm() const { return x_mm_; }
  [[nodiscard]] float y_mm() const { return y_mm_; }
  [[nodiscard]] float yaw_rate_rad_s() const { return yaw_rate_rad_s_; }
  [[nodiscard]] float bias_rad_s() const { return bias_rad_s_; }
  [[nodiscard]] bool gyro_fresh(std::uint64_t now_us) const;
  // Both yaw estimates are large and point opposite ways: one sign is wrong.
  [[nodiscard]] bool gyro_disagrees() const;

private:
  [[nodiscard]] float encoder_psi_rad() const;
  void update_wheels(const Measurements& measurements);
  void update_gyro(const Measurements& measurements);

  std::array<float, kWheelCount> counts_{};
  std::array<float, kWheelCount> counts_zero_{};
  std::array<float, kWheelCount> wheel_cps_{};
  std::array<std::array<float, kWheelCount>, kSpeedWindowTicks> ring_counts_{};
  std::array<std::uint64_t, kSpeedWindowTicks> ring_us_{};
  std::size_t ring_head_ = 0;
  std::size_t ring_filled_ = 0;

  float psi_rad_ = 0.0F;
  float x_mm_ = 0.0F;
  float y_mm_ = 0.0F;
  float s_last_counts_ = 0.0F;
  float yaw_rate_rad_s_ = 0.0F;
  float bias_rad_s_ = 0.0F;
  float last_raw_rad_s_ = 0.0F;
  std::uint32_t gyro_sequence_ = 0;
  std::uint64_t gyro_us_ = 0;
  bool gyro_seen_ = false;

  void restart_bias_block(std::uint64_t now_us);
  [[nodiscard]] bool wheels_moved_in_block() const;

  bool bias_active_ = false;
  std::uint64_t block_start_us_ = 0;
  double bias_sum_ = 0.0;
  float bias_min_ = 0.0F;
  float bias_max_ = 0.0F;
  std::uint32_t bias_samples_ = 0;
  std::array<float, kWheelCount> counts_at_block_start_{};
};

}  // namespace lf
