#pragma once

#include <array>

#include "config/robot_config.hpp"
#include "config/types.hpp"

namespace lf {

class VelocityProfiler {
public:
  VelocityProfiler();
  explicit VelocityProfiler(const ProfilerConfig& config);

  void set_config(const ProfilerConfig& config) {
    config_ = config;
    if (segment_count_ > 0) {
      optimize();
    }
  }
  [[nodiscard]] const ProfilerConfig& get_config() const { return config_; }

  void reset();
  bool add_segment(SegmentType type, float length_m, float curvature_rad_m);
  void optimize();

  void query(float current_s_m, float& out_target_speed, float& out_feedforward_steer,
             int& out_seg_idx) const;

  [[nodiscard]] int get_segment_count() const { return segment_count_; }
  [[nodiscard]] float get_total_length() const { return total_track_length_m_; }
  [[nodiscard]] bool is_optimized() const { return is_optimized_; }
  [[nodiscard]] const MapSegment& get_segment(int index) const {
    return segments_.at(static_cast<size_t>(index));
  }
  [[nodiscard]] const std::array<MapSegment, ProfilerConfig::kMaxSegments>& get_segments() const {
    return segments_;
  }

private:
  [[nodiscard]] float compute_segment_speed_at(const MapSegment& seg, float s_local) const;

  ProfilerConfig config_;
  std::array<MapSegment, ProfilerConfig::kMaxSegments> segments_{};
  int segment_count_ = 0;
  float total_track_length_m_ = 0.0f;
  bool is_optimized_ = false;
};

}  // namespace lf
