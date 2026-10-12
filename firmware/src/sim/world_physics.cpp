#include "world_physics.hpp"

#include <algorithm>
#include <cmath>

#include "sensing/normalize.hpp"

namespace lf::sim {
namespace {

// Cắt tia vào 1 khe trục (slab). Trả false nếu trượt.
bool clip_slab(float origin, float dir, float lo, float hi, float& t_enter, float& t_exit) {
  if (std::abs(dir) < 1e-9f) {
    return origin >= lo && origin <= hi;
  }
  float t1 = (lo - origin) / dir;
  float t2 = (hi - origin) / dir;
  if (t1 > t2) {
    std::swap(t1, t2);
  }
  t_enter = std::max(t_enter, t1);
  t_exit = std::min(t_exit, t2);
  return t_enter <= t_exit;
}

// Khoảng cách từ gốc tia tới AABB theo hướng (dx,dy), trong phạm vi tầm.
float ray_aabb(float ox, float oy, float dx, float dy, float min_x, float min_y, float max_x,
               float max_y, float range) {
  float t_enter = 0.0f;
  float t_exit = range;
  if (!clip_slab(ox, dx, min_x, max_x, t_enter, t_exit)) {
    return range;
  }
  if (!clip_slab(oy, dy, min_y, max_y, t_enter, t_exit)) {
    return range;
  }
  return (t_enter >= 0.0f) ? t_enter : range;
}

}

void WorldPhysics::reset(const TrackManifest& manifest) {
  manifest_ = &manifest;
  x_ = manifest.start_x_mm;
  y_ = manifest.start_y_mm;
  theta_ = manifest.start_theta_deg * kPi / 180.0f;
  v_ = 0.0f;
  w_ = 0.0f;
  wheel_ticks_l_ = 0.0f;
  wheel_ticks_r_ = 0.0f;
  ir_weights_.fill(0.0f);
  tof_front_ = sensing::kTofRangeMm;
  tof_left_ = sensing::kTofRangeMm;
  tof_right_ = sensing::kTofRangeMm;
}

void WorldPhysics::step(const lf::drive::DriveCommand& cmd, float dt) {
  const float velocity_left = cmd.left_duty * kMaxWheelSpeedMps;
  const float velocity_right = cmd.right_duty * kMaxWheelSpeedMps;
  v_ = 0.5f * (velocity_left + velocity_right);
  w_ = (velocity_right - velocity_left) / (kWheelSeparationMm / 1000.0f);
  x_ += v_ * std::cos(theta_) * dt * 1000.0f;
  y_ += v_ * std::sin(theta_) * dt * 1000.0f;
  theta_ += w_ * dt;
  wheel_ticks_l_ += velocity_left * dt * kTicksPerMeter;
  wheel_ticks_r_ += velocity_right * dt * kTicksPerMeter;
}

float WorldPhysics::cone_tof(float ox, float oy, float center_angle) const {
  if (manifest_ == nullptr) {
    return sensing::kTofRangeMm;
  }
  constexpr float kHalfFovRad = 12.5f * kPi / 180.0f;
  const std::array<float, 3> sub_angles = {
    center_angle - kHalfFovRad,
    center_angle,
    center_angle + kHalfFovRad,
  };
  float best = sensing::kTofRangeMm;
  for (const auto angle : sub_angles) {
    const float dx = std::cos(angle);
    const float dy = std::sin(angle);
    for (const auto& obstacle : manifest_->obstacles) {
      const float half_w = obstacle.width_mm * 0.5f;
      const float half_h = obstacle.height_mm * 0.5f;
      const float distance =
        ray_aabb(ox, oy, dx, dy, obstacle.x_mm - half_w, obstacle.y_mm - half_h,
                 obstacle.x_mm + half_w, obstacle.y_mm + half_h, sensing::kTofRangeMm);
      best = std::min(best, distance);
    }
  }
  return best;
}

void WorldPhysics::sample(lf::drive::Readings& frame, float dt, const TrackAsset& track) {
  frame.dt = dt;
  frame.heading_rate = w_;  // gyro thật = vận tốc góc (ticket 37)
  frame.enc_l = static_cast<std::int32_t>(wheel_ticks_l_);
  frame.enc_r = static_cast<std::int32_t>(wheel_ticks_r_);

  // IR: mảng 8 cảm biến quanh điểm nhìn trước.
  const float cos_t = std::cos(theta_);
  const float sin_t = std::sin(theta_);
  const float front_x = x_ + kSensorLookaheadMm * cos_t;
  const float front_y = y_ + kSensorLookaheadMm * sin_t;
  const float perp_x = -sin_t;  // pháp tuyến (bên phải)
  const float perp_y = cos_t;

  sensing::IrReadings ir_raw{};
  for (std::size_t i = 0; i < sensing::kIrOffsetsMm.size(); ++i) {
    const float offset = sensing::kIrOffsetsMm[i];
    const float sample_x = front_x + offset * perp_x;
    const float sample_y = front_y + offset * perp_y;
    const auto gray = static_cast<float>(track.sample_grayscale(sample_x, sample_y));
    const float weight = (gray < kLineDarkThreshold) ? (255.0f - gray) / 255.0f : 0.0f;
    ir_weights_[i] = weight;
    const auto adc_val = static_cast<std::uint16_t>(
      std::clamp((1.0f - weight) * sensing::kAdcFullScale, 0.0f, sensing::kAdcFullScale));
    ir_raw.raw[i] = adc_val;
  }
  const auto ir_result = sensing::normalize_ir(ir_raw);
  frame.line_certainty = ir_result.line_certainty;
  frame.line_offset = ir_result.line_offset;

  // ToF: 1 ở phía trước (cản trước), 2 ở hai bên sườn xe (trái -90°, phải +90°).
  const float tof_front_x = x_ + kRobotHalfLengthMm * cos_t;
  const float tof_front_y = y_ + kRobotHalfLengthMm * sin_t;
  const float tof_left_x = x_ + 15.0f * cos_t - kRobotHalfWidthMm * perp_x;
  const float tof_left_y = y_ + 15.0f * sin_t - kRobotHalfWidthMm * perp_y;
  const float tof_right_x = x_ + 15.0f * cos_t + kRobotHalfWidthMm * perp_x;
  const float tof_right_y = y_ + 15.0f * sin_t + kRobotHalfWidthMm * perp_y;

  tof_front_ = cone_tof(tof_front_x, tof_front_y, theta_);
  tof_left_ = cone_tof(tof_left_x, tof_left_y, theta_ - 0.45f);
  tof_right_ = cone_tof(tof_right_x, tof_right_y, theta_ + 0.45f);
  sensing::normalize_tof(frame, sensing::kTofRangeMm, tof_front_, tof_left_, tof_right_);

  // Finish: tâm robot vào ô đích.
  if (manifest_ != nullptr) {
    frame.finish_detected = (x_ >= manifest_->finish_min_x && x_ <= manifest_->finish_max_x &&
                             y_ >= manifest_->finish_min_y && y_ <= manifest_->finish_max_y);
  }
}

}
