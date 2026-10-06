#include "math/collision.hpp"

#include <cmath>

#include "config/constants.hpp"

namespace lf {

bool check_obb_intersection(const BoundingBox2D& box_a, const BoundingBox2D& box_b) {
  const float delta_x = box_b.center_x - box_a.center_x;
  const float delta_y = box_b.center_y - box_a.center_y;

  const float cos_a = std::cos(box_a.heading_rad);
  const float sin_a = std::sin(box_a.heading_rad);
  const float cos_b = std::cos(box_b.heading_rad);
  const float sin_b = std::sin(box_b.heading_rad);

  // Rotation difference between box B and box A
  const float delta_th = box_b.heading_rad - box_a.heading_rad;
  const float c = std::abs(std::cos(delta_th));
  const float s = std::abs(std::sin(delta_th));

  // 1. Test Box A local X-axis (heading normal)
  const float t_a1 = std::abs(delta_x * cos_a + delta_y * sin_a);
  const float r_a1 = box_a.half_width;
  const float r_b_proj_a1 = box_b.half_width * c + box_b.half_height * s;
  if (t_a1 > r_a1 + r_b_proj_a1) {
    return false;
  }

  // 2. Test Box A local Y-axis (perpendicular normal)
  const float t_a2 = std::abs(-delta_x * sin_a + delta_y * cos_a);
  const float r_a2 = box_a.half_height;
  const float r_b_proj_a2 = box_b.half_width * s + box_b.half_height * c;
  if (t_a2 > r_a2 + r_b_proj_a2) {
    return false;
  }

  // 3. Test Box B local X-axis
  const float t_b1 = std::abs(delta_x * cos_b + delta_y * sin_b);
  const float r_b1 = box_b.half_width;
  const float r_a_proj_b1 = box_a.half_width * c + box_a.half_height * s;
  if (t_b1 > r_b1 + r_a_proj_b1) {
    return false;
  }

  // 4. Test Box B local Y-axis
  const float t_b2 = std::abs(-delta_x * sin_b + delta_y * cos_b);
  const float r_b2 = box_b.half_height;
  const float r_a_proj_b2 = box_a.half_width * s + box_a.half_height * c;
  return t_b2 <= r_b2 + r_a_proj_b2;
}

bool check_robot_obstacle_collision(float robot_x, float robot_y, float robot_heading_rad,
                                    float robot_length_mm, float robot_width_mm,
                                    const ObstacleGeometry& obs) {
  const BoundingBox2D robot_box{
    robot_x, robot_y, robot_length_mm * 0.5f, robot_width_mm * 0.5f, robot_heading_rad,
  };

  constexpr float kDegToRad = constants::kPi / 180.0f;
  const float obs_heading_rad = obs.angle_deg * kDegToRad;
  const BoundingBox2D obs_box{
    obs.x_mm, obs.y_mm, obs.width_mm * 0.5f, obs.height_mm * 0.5f, obs_heading_rad,
  };

  return check_obb_intersection(robot_box, obs_box);
}

int check_robot_collision_any(float robot_x, float robot_y, float robot_heading_rad,
                              float robot_length_mm, float robot_width_mm,
                              const std::vector<ObstacleGeometry>& obstacles) {
  for (size_t i = 0; i < obstacles.size(); ++i) {
    if (check_robot_obstacle_collision(robot_x, robot_y, robot_heading_rad, robot_length_mm,
                                       robot_width_mm, obstacles.at(i))) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

}  // namespace lf
