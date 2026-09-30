#pragma once

#include <array>

#include "config/robot_config.hpp"

namespace lf {

class CentroidCalculator {
public:
  explicit CentroidCalculator(const LineSensorConfig& config);

  // Tính tọa độ trọng tâm vạch (mm). Trả về false nếu mất line.
  [[nodiscard]] bool compute(const std::array<float, LineSensorConfig::kSensorCount>& ir_raw,
                             float& out_centroid_mm) const;

  // Kiểm tra điều kiện vạch đích (toàn bộ các mắt đều đen)
  [[nodiscard]] bool is_finish_line(
    const std::array<float, LineSensorConfig::kSensorCount>& ir_raw) const;

private:
  LineSensorConfig config_;
};

}  // namespace lf
