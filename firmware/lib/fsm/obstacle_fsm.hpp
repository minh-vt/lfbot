#pragma once

#include <cstdint>

#include "config/robot_config.hpp"
#include "config/types.hpp"

namespace lf {

class ObstacleFSM {
public:
  enum class State : uint8_t { kTracking = 0, kBrakeApproach, kArcChicane, kEmergencySearch };

  ObstacleFSM();
  explicit ObstacleFSM(const ObstacleConfig& config);

  void reset();
  [[nodiscard]] bool is_bypassing() const { return state_ != State::kTracking; }
  [[nodiscard]] State get_state() const { return state_; }
  [[nodiscard]] float get_bypass_direction() const { return bypass_direction_; }
  [[nodiscard]] float get_progress_dist() const { return progress_dist_m_; }

  void set_config(const ObstacleConfig& config) { config_ = config; }
  [[nodiscard]] const ObstacleConfig& get_config() const { return config_; }

  void update(const SensorData& in, float current_speed, MotorCmd& cmd, bool any_ir_on_line);

private:
  ObstacleConfig config_;
  State state_ = State::kTracking;
  float timer_s_ = 0.0f;
  float bypass_direction_ = 1.0f;  // +1.0f (re phai), -1.0f (re trai)
  float progress_dist_m_ = 0.0f;
};

}  // namespace lf
