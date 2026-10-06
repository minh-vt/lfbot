#pragma once

#include <vector>

#include "config/types.hpp"

namespace lf {

struct BoundingBox2D {
  float center_x = 0.0f;
  float center_y = 0.0f;
  float half_width = 0.0f;
  float half_height = 0.0f;
  float heading_rad = 0.0f;
};

// Separating Axis Theorem (SAT) collision test between two 2D oriented bounding boxes (OBBs).
// Returns true if box_a and box_b overlap/intersect.
bool check_obb_intersection(const BoundingBox2D& box_a, const BoundingBox2D& box_b);

// Checks if a robot with given center, dimensions, and heading collides with an obstacle.
bool check_robot_obstacle_collision(float robot_x, float robot_y, float robot_heading_rad,
                                    float robot_length_mm, float robot_width_mm,
                                    const ObstacleGeometry& obs);

// Checks if the robot collides with any obstacle in the list.
// Returns the index of the first collided obstacle, or -1 if no collision.
int check_robot_collision_any(float robot_x, float robot_y, float robot_heading_rad,
                              float robot_length_mm, float robot_width_mm,
                              const std::vector<ObstacleGeometry>& obstacles);

}  // namespace lf
