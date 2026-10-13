#include "world_physics.hpp"

#include <algorithm>
#include <array>
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
  tof_front_ = layout_.tof_range_mm;
  tof_left_ = layout_.tof_range_mm;
  tof_right_ = layout_.tof_range_mm;
}

void WorldPhysics::step(const lf::drive::DriveCommand& cmd, float dt) {
  const float velocity_left = cmd.left_duty * kMaxWheelSpeedMps;
  const float velocity_right = cmd.right_duty * kMaxWheelSpeedMps;
  v_ = 0.5f * (velocity_left + velocity_right);
  w_ = (velocity_right - velocity_left) / (layout_.wheelbase_mm / 1000.0f);
  x_ += v_ * std::cos(theta_) * dt * 1000.0f;
  y_ += v_ * std::sin(theta_) * dt * 1000.0f;
  theta_ += w_ * dt;
  wheel_ticks_l_ += velocity_left * dt * sensing::ticks_per_meter(layout_);
  wheel_ticks_r_ += velocity_right * dt * sensing::ticks_per_meter(layout_);
}

float WorldPhysics::cone_tof(float ox, float oy, float center_angle) const {
  if (manifest_ == nullptr) {
    return layout_.tof_range_mm;
  }
  const float half_fov_rad = layout_.tof_fov_deg * kPi / 180.0f;
  const std::array<float, 3> sub_angles = {
    center_angle - half_fov_rad,
    center_angle,
    center_angle + half_fov_rad,
  };
  float best = layout_.tof_range_mm;
  for (const auto angle : sub_angles) {
    const float dx = std::cos(angle);
    const float dy = std::sin(angle);
    for (const auto& obstacle : manifest_->obstacles) {
      const float half_w = obstacle.width_mm * 0.5f;
      const float half_h = obstacle.height_mm * 0.5f;
      const float distance =
        ray_aabb(ox, oy, dx, dy, obstacle.x_mm - half_w, obstacle.y_mm - half_h,
                 obstacle.x_mm + half_w, obstacle.y_mm + half_h, layout_.tof_range_mm);
      best = std::min(best, distance);
    }
  }
  return best;
}

void WorldPhysics::sample(lf::drive::Readings& frame, float dt, const TrackAsset& track) {
  frame.dt = dt;
  frame.enc_l = static_cast<std::int32_t>(wheel_ticks_l_);
  frame.enc_r = static_cast<std::int32_t>(wheel_ticks_r_);

  // IR: vị trí từng cảm biến lấy từ layout (thẳng hoặc cong).
  const float cos_t = std::cos(theta_);
  const float sin_t = std::sin(theta_);
  const float perp_x = -sin_t;  // pháp tuyến (bên phải)
  const float perp_y = cos_t;

  sensing::IrReadings ir_raw{};
  for (std::size_t i = 0; i < layout_.ir_lateral_mm.size(); ++i) {
    const float forward = layout_.ir_forward_mm[i];
    const float lateral = layout_.ir_lateral_mm[i];
    const float sample_x = x_ + forward * cos_t + lateral * perp_x;
    const float sample_y = y_ + forward * sin_t + lateral * perp_y;
    const auto gray = static_cast<float>(track.sample_grayscale(sample_x, sample_y));
    const float weight = (gray < kLineDarkThreshold) ? (255.0f - gray) / 255.0f : 0.0f;
    ir_weights_[i] = weight;
    const auto adc_val = static_cast<std::uint16_t>(
      std::clamp((1.0f - weight) * sensing::kAdcFullScale, 0.0f, sensing::kAdcFullScale));
    ir_raw.raw[i] = adc_val;
  }
  const auto ir_result = sensing::normalize_ir(ir_raw, layout_);
  frame.line_certainty = ir_result.line_certainty;
  frame.line_offset = ir_result.line_offset;

  // ToF: 1 phía trước, 2 hai bên sườn (góc đọc từ layout — T04 đo để chọn).
  const float side_angle = layout_.tof_side_angle_deg * kPi / 180.0f;
  const float tof_front_x = x_ + layout_.half_length_mm * cos_t;
  const float tof_front_y = y_ + layout_.half_length_mm * sin_t;
  const float tof_side_fwd = layout_.tof_side_forward_mm;
  const float tof_left_x = x_ + tof_side_fwd * cos_t - layout_.half_width_mm * perp_x;
  const float tof_left_y = y_ + tof_side_fwd * sin_t - layout_.half_width_mm * perp_y;
  const float tof_right_x = x_ + tof_side_fwd * cos_t + layout_.half_width_mm * perp_x;
  const float tof_right_y = y_ + tof_side_fwd * sin_t + layout_.half_width_mm * perp_y;

  tof_front_ = cone_tof(tof_front_x, tof_front_y, theta_);
  tof_left_ = cone_tof(tof_left_x, tof_left_y, theta_ - side_angle);
  tof_right_ = cone_tof(tof_right_x, tof_right_y, theta_ + side_angle);
  sensing::normalize_tof(frame, layout_, tof_front_, tof_left_, tof_right_);

  // Finish: tâm robot vào ô đích.
  if (manifest_ != nullptr) {
    frame.finish_detected = (x_ >= manifest_->finish_min_x && x_ <= manifest_->finish_max_x &&
                             y_ >= manifest_->finish_min_y && y_ <= manifest_->finish_max_y);
  }
}

}
