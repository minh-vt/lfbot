#include "world_physics.hpp"

#include <algorithm>
#include <cmath>

#include "sensing/sensor_model.hpp"

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
  manifest_ = manifest;
  x_ = manifest.start_x_mm;
  y_ = manifest.start_y_mm;
  theta_ = manifest.start_theta_deg * kPi / 180.0f;
  v_ = 0.0f;
  w_ = 0.0f;
  wheel_ticks_l_ = 0.0f;
  wheel_ticks_r_ = 0.0f;
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

float WorldPhysics::ray_tof(float ox, float oy, float base_theta, float offset) const {
  const float angle = base_theta + offset;
  const float dx = std::cos(angle);
  const float dy = std::sin(angle);
  float best = sensing::kTofRangeMm;
  for (const auto& obstacle : manifest_.obstacles) {
    const float half_w = obstacle.width_mm * 0.5f;
    const float half_h = obstacle.height_mm * 0.5f;
    const float distance =
      ray_aabb(ox, oy, dx, dy, obstacle.x_mm - half_w, obstacle.y_mm - half_h,
               obstacle.x_mm + half_w, obstacle.y_mm + half_h, sensing::kTofRangeMm);
    best = std::min(best, distance);
  }
  return best;
}

void WorldPhysics::sample(lf::drive::Readings& frame, float dt, const TrackAsset& track) const {
  frame.dt = dt;
  frame.enc_l = static_cast<std::int32_t>(wheel_ticks_l_);
  frame.enc_r = static_cast<std::int32_t>(wheel_ticks_r_);

  // IR: mảng 8 cảm biến quanh điểm nhìn trước.
  const float front_x = x_ + kSensorLookaheadMm * std::cos(theta_);
  const float front_y = y_ + kSensorLookaheadMm * std::sin(theta_);
  const float perp_x = -std::sin(theta_);  // pháp tuyến (bên phải)
  const float perp_y = std::cos(theta_);

  float weight_sum = 0.0f;
  float weighted_pos = 0.0f;
  for (const float offset : sensing::kIrOffsetsMm) {
    const float sample_x = front_x + offset * perp_x;
    const float sample_y = front_y + offset * perp_y;
    const auto gray = static_cast<float>(track.sample_grayscale(sample_x, sample_y));
    const float weight = (gray < kLineDarkThreshold) ? (255.0f - gray) / 255.0f : 0.0f;
    weight_sum += weight;
    weighted_pos += weight * offset;
  }
  frame.line_certainty = std::clamp(weight_sum, 0.0f, 1.0f);
  frame.line_offset = 0.0f;
  if (weight_sum >= 1e-3f) {
    frame.line_offset =
      std::clamp((weighted_pos / weight_sum) / sensing::kIrHalfSpanMm, -1.0f, 1.0f);
  }

  // ToF: 3 tia (front, trái, phải).
  const float distance_front = ray_tof(front_x, front_y, theta_, 0.0f);
  const float distance_left = ray_tof(front_x, front_y, theta_, -0.4f);
  const float distance_right = ray_tof(front_x, front_y, theta_, 0.4f);
  sensing::normalize_tof(frame, sensing::kTofRangeMm, distance_front, distance_left,
                         distance_right);

  // Finish: tâm robot vào ô đích.
  frame.finish_detected = (x_ >= manifest_.finish_min_x && x_ <= manifest_.finish_max_x &&
                           y_ >= manifest_.finish_min_y && y_ <= manifest_.finish_max_y);
}

}
