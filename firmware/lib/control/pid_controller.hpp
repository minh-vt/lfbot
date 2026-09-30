#pragma once

#include "config/robot_config.hpp"

namespace lf {

class PIDController {
public:
  PIDController();
  explicit PIDController(const PIDConfig& config);

  void set_config(const PIDConfig& config);
  void set_gains(float kp, float ki, float kd);
  void reset();

  float update(float error, float dt);
  float update_speed_scaled(float error, float dt, float current_speed, float ref_speed = 1.0f);

private:
  PIDConfig config_{};
  float integral_ = 0.0f;
  float prev_error_ = 0.0f;
  float prev_d_term_ = 0.0f;
};

}  // namespace lf
