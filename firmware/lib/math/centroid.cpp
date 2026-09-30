#include "math/centroid.hpp"

#include <algorithm>

namespace lf {

CentroidCalculator::CentroidCalculator(const LineSensorConfig& config) : config_(config) {}

bool CentroidCalculator::compute(const std::array<float, LineSensorConfig::kSensorCount>& ir_raw,
                                 float& out_centroid_mm) const {
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

  out_centroid_mm = weighted_sum / sum_values;
  return true;
}

bool CentroidCalculator::is_finish_line(
  const std::array<float, LineSensorConfig::kSensorCount>& ir_raw) const {
  int black_eyes_count = 0;
  for (size_t i = 0; i < LineSensorConfig::kSensorCount; ++i) {
    if (ir_raw.at(i) >= config_.finish_line_threshold) {
      black_eyes_count++;
    }
  }

  return black_eyes_count >= config_.min_finish_eyes;
}

}  // namespace lf
