#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace lf::sensing {

// SSoT hình học robot + cảm biến (đơn vị mm). Sim (physics + renderer), drive tuning
// và esp32 đều đọc từ đây: đổi robot một chỗ, cả hệ thống cập nhật theo.
struct RobotLayout {
  // Thân robot.
  float wheelbase_mm = 138.0f;   // khoảng cách 2 bánh (vi sai) = chiều rộng thân
  float half_length_mm = 72.5f;  // robot 145mm
  float half_width_mm = 69.0f;   // robot 138mm

  // Cụm IR dò vạch: vị trí TỪNG cảm biến trong hệ robot (dương = trước/phải).
  // Thẳng: mọi forward bằng nhau. Cong: cảm biến ngoài lệch forward hơn.
  std::array<float, 8> ir_lateral_mm = {-42.0f, -30.0f, -18.0f, -6.0f, 6.0f, 18.0f, 30.0f, 42.0f};
  std::array<float, 8> ir_forward_mm = {55.0f, 55.0f, 55.0f, 55.0f, 55.0f, 55.0f, 55.0f, 55.0f};

  // ToF: 1 trước + 2 sườn.
  float tof_range_mm = 500.0f;        // tầm hữu dụng
  float tof_fov_deg = 25.0f;          // nửa FoV mỗi nón
  float tof_side_angle_deg = 45.0f;   // góc ToF sườn so với heading (T04 đo để chọn)
  float tof_side_forward_mm = 15.0f;  // ToF sườn đặt lệch trước bao nhiêu mm

  float tick_length_mm = 20.0f;  // 1 tick odometry = 20mm
};

// Các hằng SUY RA — không ai được tay tính lại.

[[nodiscard]] constexpr float ir_half_span_mm(const RobotLayout& layout) {
  return layout.ir_lateral_mm[layout.ir_lateral_mm.size() - 1];
}

[[nodiscard]] inline float robot_radius_mm(const RobotLayout& layout) {
  // Bán kính bao = nửa đường chéo thân.
  return std::sqrt(layout.half_length_mm * layout.half_length_mm +
                   layout.half_width_mm * layout.half_width_mm);
}

[[nodiscard]] constexpr float ticks_per_meter(const RobotLayout& layout) {
  return 1000.0f / layout.tick_length_mm;
}

// Chân lý không-thứ-nguyên cho drive (đơn vị tick).
[[nodiscard]] constexpr float geometry_l(const RobotLayout& layout) {
  return layout.wheelbase_mm / layout.tick_length_mm;
}

[[nodiscard]] constexpr float geometry_s(const RobotLayout& layout) {
  return ir_half_span_mm(layout) / layout.tick_length_mm;
}

// Layout hiện tại của robot — nguồn chân lý duy nhất trong repo.
inline constexpr RobotLayout kRobotLayout{};

}
