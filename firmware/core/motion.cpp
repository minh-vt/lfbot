#include "motion.hpp"

#include <algorithm>
#include <cmath>

namespace lf {
namespace {

// A gap longer than this between gyro samples is a dropout, not motion to integrate.
constexpr std::uint64_t kMaxGyroGapUs = 20000;
// Fewer samples than this cannot show the gyro is steady.
constexpr std::uint32_t kMinBiasSamples = 50;
constexpr float kMicrosecondsPerSecond = 1.0e6F;

}  // namespace

void Motion::update(const Measurements& measurements) {
  update_wheels(measurements);
  update_gyro(measurements);
  // Each tick's few millimetres are short enough to take along the latest heading.
  const float s = s_counts();
  const float step_mm = (s - s_last_counts_) * kMmPerCount;
  s_last_counts_ = s;
  x_mm_ += step_mm * std::cos(psi_rad_);
  y_mm_ += step_mm * std::sin(psi_rad_);
}

void Motion::update_wheels(const Measurements& measurements) {
  if (measurements.encoders.status != SampleStatus::kValid) {
    return;
  }
  for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
    const auto raw = static_cast<float>(measurements.encoders.value[wheel]);
    counts_[wheel] = kInvertEncoder[wheel] ? -raw : raw;
  }
  ring_counts_[ring_head_] = counts_;
  ring_us_[ring_head_] = measurements.encoders.latest_us;
  ring_head_ = (ring_head_ + 1) % kSpeedWindowTicks;
  ring_filled_ = std::min(ring_filled_ + 1, kSpeedWindowTicks);
  if (ring_filled_ < 2) {
    return;
  }
  const std::size_t oldest = (ring_head_ + kSpeedWindowTicks - ring_filled_) % kSpeedWindowTicks;
  const std::size_t newest = (ring_head_ + kSpeedWindowTicks - 1) % kSpeedWindowTicks;
  const auto elapsed_us = static_cast<float>(ring_us_[newest] - ring_us_[oldest]);
  if (elapsed_us <= 0.0F) {
    return;
  }
  for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
    wheel_cps_[wheel] = (ring_counts_[newest][wheel] - ring_counts_[oldest][wheel]) *
                        kMicrosecondsPerSecond / elapsed_us;
  }
}

void Motion::update_gyro(const Measurements& measurements) {
  const Sample<float>& gyro = measurements.gyro_z_rad_s;
  if (gyro.status != SampleStatus::kValid || gyro.sequence == gyro_sequence_) {
    return;
  }
  gyro_sequence_ = gyro.sequence;
  const float raw = gyro.value;
  if (bias_active_) {
    bias_sum_ += static_cast<double>(raw);
    bias_min_ = bias_samples_ == 0 ? raw : std::min(bias_min_, raw);
    bias_max_ = bias_samples_ == 0 ? raw : std::max(bias_max_, raw);
    ++bias_samples_;
  }
  yaw_rate_rad_s_ = kGyroSign * (raw - bias_rad_s_);
  if (gyro_seen_ && gyro.latest_us > gyro_us_ && gyro.latest_us - gyro_us_ <= kMaxGyroGapUs) {
    const float dt = static_cast<float>(gyro.latest_us - gyro_us_) / kMicrosecondsPerSecond;
    // Trapezoid between this sample and the previous one.
    psi_rad_ +=
      0.5F * (kGyroSign * (raw - bias_rad_s_) + kGyroSign * (last_raw_rad_s_ - bias_rad_s_)) * dt;
  }
  last_raw_rad_s_ = raw;
  gyro_us_ = gyro.latest_us;
  gyro_seen_ = true;
}

float Motion::s_counts() const {
  return 0.5F * ((counts_[0] - counts_zero_[0]) + (counts_[1] - counts_zero_[1]));
}

float Motion::encoder_psi_rad() const {
  const float difference = (counts_[1] - counts_zero_[1]) - (counts_[0] - counts_zero_[0]);
  return difference / kTrackCountsPerRad;
}

void Motion::zero() {
  counts_zero_ = counts_;
  psi_rad_ = 0.0F;
  x_mm_ = 0.0F;
  y_mm_ = 0.0F;
  s_last_counts_ = 0.0F;
}

void Motion::begin_bias() {
  bias_active_ = true;
  block_start_us_ = 0;
}

void Motion::restart_bias_block(std::uint64_t now_us) {
  block_start_us_ = now_us;
  bias_sum_ = 0.0;
  bias_samples_ = 0;
  counts_at_block_start_ = counts_;
}

bool Motion::wheels_moved_in_block() const {
  for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
    if (std::abs(counts_[wheel] - counts_at_block_start_[wheel]) >
        static_cast<float>(kStillCounts)) {
      return true;
    }
  }
  return false;
}

bool Motion::update_bias(std::uint64_t now_us) {
  if (block_start_us_ == 0) {
    restart_bias_block(now_us);
    return false;
  }
  if (now_us - block_start_us_ < kBiasBlockUs) {
    return false;
  }
  const bool still = bias_samples_ >= kMinBiasSamples && bias_max_ - bias_min_ <= kStillRateRad &&
                     !wheels_moved_in_block();
  if (!still) {
    restart_bias_block(now_us);
    return false;
  }
  bias_rad_s_ = static_cast<float>(bias_sum_ / static_cast<double>(bias_samples_));
  bias_active_ = false;
  return true;
}

bool Motion::gyro_fresh(std::uint64_t now_us) const {
  return gyro_seen_ && now_us >= gyro_us_ && now_us - gyro_us_ <= kGyroStaleUs;
}

bool Motion::gyro_disagrees() const {
  const float encoder = encoder_psi_rad();
  return psi_rad_ * encoder < 0.0F && std::abs(psi_rad_) > kYawDisagreeRad &&
         std::abs(encoder) > kYawDisagreeRad;
}

}  // namespace lf
