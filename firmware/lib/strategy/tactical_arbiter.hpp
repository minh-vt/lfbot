#pragma once

#include <cstdint>

#include "config/robot_config.hpp"
#include "config/types.hpp"

namespace lf {

struct TacticalReference {
  float lateral_error_m = 0.0f;     // Effective lateral error (optical centroid + APF offset)
  float target_v_mps = 0.0f;        // Target speed commanded by strategy
  float steer_cmd_rad = 0.0f;       // Direct tactical steering command during active evasion
  bool has_steer_override = false;  // True if steer_cmd_rad overrides lateral PID
  bool should_halt = false;         // True if line is lost beyond timeout or fatal obstacle
  bool is_evading = false;          // True if obstacle evasion chicane is active
  bool is_dead_reckoning = false;   // True if maintaining momentum across optical gap
  uint8_t bypass_state = 0;         // 0: normal, 1: evading/bypassing
};

class TacticalBehaviorArbiter {
public:
  TacticalBehaviorArbiter(const ObstacleConfig& obs_cfg, const ProfilerConfig& prof_cfg,
                          const TrackingConfig& track_cfg);

  void reset();

  TacticalReference update(const Pose2D& pose, float current_v_mps, float centroid_mm,
                           bool line_found, float tof_front_m, float tof_left_m, float tof_right_m,
                           float dt, RobotMode mode);

  [[nodiscard]] float get_line_loss_duration_s() const { return line_loss_duration_s_; }
  [[nodiscard]] bool is_evading() const { return is_evading_; }

  void set_obstacle_config(const ObstacleConfig& cfg) { obs_cfg_ = cfg; }
  [[nodiscard]] const ObstacleConfig& get_obstacle_config() const { return obs_cfg_; }

  void set_profiler_config(const ProfilerConfig& cfg) { prof_cfg_ = cfg; }
  [[nodiscard]] const ProfilerConfig& get_profiler_config() const { return prof_cfg_; }

  void set_tracking_config(const TrackingConfig& cfg) { track_cfg_ = cfg; }
  [[nodiscard]] const TrackingConfig& get_tracking_config() const { return track_cfg_; }

private:
  void trigger_obstacle_evasion(const Pose2D& pose, float tof_front_m, float tof_left_m,
                                float tof_right_m);

  ObstacleConfig obs_cfg_;
  ProfilerConfig prof_cfg_;
  TrackingConfig track_cfg_;

  float line_loss_duration_s_ = 0.0f;
  float last_valid_lateral_error_m_ = 0.0f;

  bool is_evading_ = false;
  float evasion_origin_x_ = 0.0f;
  float evasion_origin_y_ = 0.0f;
  float evasion_origin_th_ = 0.0f;
  float evasion_side_sign_ = 1.0f;
  float current_bypass_length_m_ = 0.82f;
  float current_bypass_speed_mps_ = 0.45f;
  float recovery_timer_s_ = 0.0f;
};

}  // namespace lf
