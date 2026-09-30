#include "control/velocity_profiler.hpp"

#include <algorithm>
#include <cmath>

namespace lf {

VelocityProfiler::VelocityProfiler() : config_(ProfilerConfig{}) {
  reset();
}

VelocityProfiler::VelocityProfiler(const ProfilerConfig& config) : config_(config) {
  reset();
}

void VelocityProfiler::reset() {
  segment_count_ = 0;
  total_track_length_m_ = 0.0f;
  is_optimized_ = false;
}

bool VelocityProfiler::add_segment(SegmentType type, float length_m, float curvature_rad_m) {
  // Guard clause: Vượt quá dung lượng mảng hoặc chiều dài quá nhỏ
  if (segment_count_ >= ProfilerConfig::kMaxSegments || length_m <= 0.01f) {
    return false;
  }

  MapSegment& seg = segments_.at(static_cast<size_t>(segment_count_));
  seg.type = type;
  seg.start_dist_m = total_track_length_m_;
  seg.length_m = length_m;
  seg.curvature_rad_m = curvature_rad_m;
  seg.v_max_allowed = config_.safe_map_speed_mps;
  seg.v_entry_target = 0.0f;
  seg.v_exit_target = 0.0f;

  total_track_length_m_ += length_m;
  segment_count_++;
  return true;
}

void VelocityProfiler::optimize() {
  // Guard clause: Không có dữ liệu sa bàn hoặc vượt giới hạn
  if (segment_count_ <= 0 || segment_count_ > ProfilerConfig::kMaxSegments) {
    return;
  }

  const float g = config_.gravity_mps2;
  const float mu = config_.corner_grip_mu;
  const float a_brake = config_.max_brake_mps2;
  const float a_accel = config_.max_accel_mps2;
  const float v_top = config_.max_straight_speed_mps;

  // Bước 1: Giới hạn vật lý từng góc cua (Curvature Speed Ceiling)
  for (int i = 0; i < segment_count_; ++i) {
    MapSegment& seg = segments_.at(static_cast<size_t>(i));

    if (seg.type == SegmentType::kStraight) {
      seg.v_max_allowed = v_top;
      continue;
    }

    if (seg.type == SegmentType::kObstacleZone) {
      seg.v_max_allowed = config_.obstacle_zone_speed_mps;
      continue;
    }

    if (seg.type == SegmentType::kFinishZone) {
      seg.v_max_allowed = config_.safe_map_speed_mps;
      continue;
    }

    const float r = (seg.curvature_rad_m > 0.001f) ? (1.0f / seg.curvature_rad_m)
                                                   : config_.fallback_curve_radius_m;
    const float v_grip = std::sqrt(mu * g * r);
    seg.v_max_allowed = std::min(v_grip, v_top);
  }

  // Bước 2: Quét lùi từ đích về xuất phát (Backward Pass - Tìm điểm phanh tối ưu)
  const auto last_idx = static_cast<size_t>(segment_count_ - 1);
  segments_.at(last_idx).v_exit_target = 0.0f;
  for (int i = segment_count_ - 1; i >= 0; --i) {
    MapSegment& seg = segments_.at(static_cast<size_t>(i));
    const float v_exit = seg.v_exit_target;
    const float v_entry_limit = std::sqrt(v_exit * v_exit + 2.0f * a_brake * seg.length_m);
    seg.v_entry_target = std::min(seg.v_max_allowed, v_entry_limit);

    if (i > 0) {
      const auto prev_idx = static_cast<size_t>(i - 1);
      segments_.at(prev_idx).v_exit_target = seg.v_entry_target;
    }
  }

  // Bước 3: Quét xuôi từ xuất phát đến đích (Forward Pass - Tăng tốc kịch kim)
  // Khởi động với v_launch tối thiểu để triệt tiêu hoàn toàn lỗi đứng yên lắc lư tại s = 0
  const float v_launch = std::min(config_.launch_min_speed_mps, segments_.at(0).v_max_allowed);
  segments_.at(0).v_entry_target = std::max(v_launch, segments_.at(0).v_entry_target);
  for (int i = 0; i < segment_count_; ++i) {
    MapSegment& seg = segments_.at(static_cast<size_t>(i));
    const float v_entry = seg.v_entry_target;
    const float v_accel_limit = std::sqrt(v_entry * v_entry + 2.0f * a_accel * seg.length_m);
    seg.v_exit_target = std::min({v_accel_limit, seg.v_max_allowed, seg.v_exit_target});

    if (i + 1 < segment_count_) {
      const size_t next_idx = static_cast<size_t>(i) + 1U;
      segments_.at(next_idx).v_entry_target =
        std::min(segments_.at(next_idx).v_entry_target, seg.v_exit_target);
    }
  }

  is_optimized_ = true;
}

float VelocityProfiler::compute_segment_speed_at(const MapSegment& seg, float s_local) const {
  const float v_entry = seg.v_entry_target;
  const float v_exit = seg.v_exit_target;
  const float v_max = seg.v_max_allowed;
  const float a_accel = config_.max_accel_mps2;
  const float a_brake = config_.max_brake_mps2;

  const float d_accel = std::max(0.0f, (v_max * v_max - v_entry * v_entry) / (2.0f * a_accel));
  const float d_decel = std::max(0.0f, (v_max * v_max - v_exit * v_exit) / (2.0f * a_brake));

  // Trường hợp đoạn đường đủ dài để đạt vận tốc tối đa v_max
  if (d_accel + d_decel <= seg.length_m) {
    if (s_local <= d_accel) {
      return std::sqrt(v_entry * v_entry + 2.0f * a_accel * s_local);
    }
    if (s_local <= seg.length_m - d_decel) {
      return v_max;
    }
    const float dist_from_end = seg.length_m - s_local;
    return std::sqrt(v_exit * v_exit + 2.0f * a_brake * dist_from_end);
  }

  // Trường hợp tam giác vận tốc (không đủ chiều dài đạt v_max)
  const float k = 1.0f / (2.0f * a_accel) + 1.0f / (2.0f * a_brake);
  const float v_peak_sq =
    (seg.length_m + (v_entry * v_entry) / (2.0f * a_accel) + (v_exit * v_exit) / (2.0f * a_brake)) /
    k;
  const float d_accel_peak = (v_peak_sq - v_entry * v_entry) / (2.0f * a_accel);

  if (s_local <= d_accel_peak) {
    return std::sqrt(v_entry * v_entry + 2.0f * a_accel * s_local);
  }

  const float dist_from_end = seg.length_m - s_local;
  return std::sqrt(v_exit * v_exit + 2.0f * a_brake * dist_from_end);
}

void VelocityProfiler::query(float current_s_m, float& out_target_speed,
                             float& out_feedforward_steer, int& out_seg_idx) const {
  // Guard clause: Chưa tối ưu hóa hoặc mảng rỗng
  if (!is_optimized_ || segment_count_ == 0) {
    out_target_speed = config_.safe_map_speed_mps;
    out_feedforward_steer = 0.0f;
    out_seg_idx = 0;
    return;
  }

  // Tìm phân đoạn theo quãng đường thực tế
  int current_idx = 0;
  for (int i = 0; i < segment_count_; ++i) {
    if (current_s_m >= segments_.at(static_cast<size_t>(i)).start_dist_m &&
        current_s_m < segments_.at(static_cast<size_t>(i)).start_dist_m +
                        segments_.at(static_cast<size_t>(i)).length_m) {
      current_idx = i;
      break;
    }
    if (i == segment_count_ - 1) {
      current_idx = i;
    }
  }

  out_seg_idx = current_idx;
  const MapSegment& seg = segments_.at(static_cast<size_t>(current_idx));

  const float s_local = std::clamp(current_s_m - seg.start_dist_m, 0.0f, seg.length_m);
  out_target_speed = compute_segment_speed_at(seg, s_local);

  // Nhìn trước một khoảng lookahead để tính bù lái trước khúc cua
  const float lookahead_s = current_s_m + config_.lookahead_distance_m;
  out_feedforward_steer = 0.0f;

  for (int j = current_idx; j < segment_count_; ++j) {
    const auto& check_seg = segments_.at(static_cast<size_t>(j));
    if (lookahead_s >= check_seg.start_dist_m &&
        lookahead_s < check_seg.start_dist_m + check_seg.length_m) {
      if (check_seg.type == SegmentType::kCurveLeft) {
        out_feedforward_steer = -check_seg.curvature_rad_m * 0.15f;
      } else if (check_seg.type == SegmentType::kCurveRight) {
        out_feedforward_steer = check_seg.curvature_rad_m * 0.15f;
      }
      break;
    }
  }
}

}  // namespace lf
