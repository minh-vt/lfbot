#pragma once

#include <array>
#include <cstdint>

#include "config/robot_config.hpp"

namespace lf {

enum class BranchPriority : uint8_t {
  kCenter = 0,  // Standard weighted centroid across all sensors
  kLeftFirst,   // Ancient micromouse rule: prioritize leftmost line branch
  kRightFirst,  // Prioritize rightmost line branch
};

class CentroidCalculator {
public:
  explicit CentroidCalculator(const LineSensorConfig& config);

  // Tính tọa độ trọng tâm vạch (mm). Trả về false nếu mất line.
  [[nodiscard]] bool compute(const std::array<float, LineSensorConfig::kSensorCount>& ir_raw,
                             float& out_centroid_mm,
                             BranchPriority priority = BranchPriority::kCenter) const;

  // Nhan dien vach ngang/nga tu/khối ô vuông (static để không phụ thuộc trạng thái)
  [[nodiscard]] static bool is_cross_marker(
    const std::array<float, LineSensorConfig::kSensorCount>& ir_raw, float threshold = 0.45f,
    int min_eyes = 6);

  // Tính độ tin cậy quang học liên tục gamma in [0, 1]
  [[nodiscard]] static float compute_optical_confidence(float total_intensity, float threshold,
                                                        float k = 10.0f);

private:
  LineSensorConfig config_;
};

}  // namespace lf
