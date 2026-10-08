#include "math/centroid.hpp"

#include <algorithm>

namespace lf {

namespace {

constexpr int kMaxSingleLineSensors = 3;  // A nominal 20mm racing line spans at most 3 sensors

bool compute_directional_leading_cluster(
  const std::array<float, LineSensorConfig::kSensorCount>& ir_raw,
  const std::array<float, LineSensorConfig::kSensorCount>& weights_mm, float threshold,
  bool scan_left_to_right, float& out_centroid_mm) {
  constexpr int kCount = static_cast<int>(LineSensorConfig::kSensorCount);
  const int step = scan_left_to_right ? 1 : -1;
  const int init = scan_left_to_right ? 0 : kCount - 1;

  int first_start = -1;
  int gap_found = 0;
  bool second_cluster = false;
  int first_cluster_len = 0;

  for (int idx = 0; idx < kCount; ++idx) {
    const int i = init + idx * step;
    if (ir_raw.at(static_cast<size_t>(i)) >= threshold) {
      if (first_start < 0) {
        first_start = i;
        first_cluster_len = 1;
      } else if (gap_found == 0) {
        first_cluster_len++;
      } else {
        second_cluster = true;
        break;
      }
    } else if (first_start >= 0) {
      gap_found = 1;
    }
  }

  // A junction, fork, or solid box occurs if:
  // 1. There is a second cluster (e.g. Y-split fork with white gap), OR
  // 2. The cluster is wider than a single line (> 3 sensors, e.g. solid black box / T-bar)
  const bool is_junction_or_box = second_cluster || (first_cluster_len > kMaxSingleLineSensors);

  if (is_junction_or_box && first_start >= 0) {
    float sub_w = 0.0f;
    float sub_v = 0.0f;
    // Cap cluster evaluation to nominal line width (at most 3 sensors from priority leading edge)
    const int actual_end =
      first_start + (std::min(first_cluster_len, kMaxSingleLineSensors) - 1) * step;
    const int min_idx = std::min(first_start, actual_end);
    const int max_idx = std::max(first_start, actual_end);

    for (int i = min_idx; i <= max_idx; ++i) {
      const float val = std::clamp(ir_raw.at(static_cast<size_t>(i)), 0.0f, 1.0f);
      sub_w += weights_mm.at(static_cast<size_t>(i)) * val;
      sub_v += val;
    }

    if (sub_v > 0.01f) {
      out_centroid_mm = sub_w / sub_v;
      return true;
    }
  }

  return false;
}

}  // namespace

CentroidCalculator::CentroidCalculator(const LineSensorConfig& config) : config_(config) {}

bool CentroidCalculator::compute(const std::array<float, LineSensorConfig::kSensorCount>& ir_raw,
                                 float& out_centroid_mm, BranchPriority priority) const {
  float weighted_sum = 0.0f;
  float sum_values = 0.0f;

  for (size_t i = 0; i < LineSensorConfig::kSensorCount; ++i) {
    const float val = std::clamp(ir_raw.at(i), 0.0f, 1.0f);
    weighted_sum += config_.weights_mm.at(i) * val;
    sum_values += val;
  }

  // Guard clause: Nếu tổng cường độ quá nhỏ -> Xe trượt hoàn toàn khỏi line
  if (sum_values < config_.line_presence_threshold) {
    return false;
  }

  // Ancient Micromouse / First-Principles Directional Branch Selection
  if (priority == BranchPriority::kLeftFirst &&
      compute_directional_leading_cluster(
        ir_raw, config_.weights_mm, config_.branch_cluster_threshold, true, out_centroid_mm)) {
    return true;
  }
  if (priority == BranchPriority::kRightFirst &&
      compute_directional_leading_cluster(
        ir_raw, config_.weights_mm, config_.branch_cluster_threshold, false, out_centroid_mm)) {
    return true;
  }

  // Normal single continuous line centroid
  out_centroid_mm = weighted_sum / sum_values;
  return true;
}

float CentroidCalculator::compute_optical_confidence(float total_intensity, float threshold,
                                                     float k) {
  const float diff = total_intensity - threshold;
  return 1.0f / (1.0f + std::exp(-k * diff));
}

}  // namespace lf
