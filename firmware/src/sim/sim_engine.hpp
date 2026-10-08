#pragma once

#include <array>
#include <deque>
#include <iosfwd>
#include <string>
#include <vector>

#include "config/robot_config.hpp"
#include "config/types.hpp"
#include "math/kinematics.hpp"
#include "race_arbiter.hpp"
#include "raycast.hpp"
#include "raylib.h"
#include "system/robot_coordinator.hpp"
#include "system/telemetry_packet.hpp"
#include "track_manifest.hpp"
#include "types.hpp"

namespace lf::sim {

struct SimStepResult {
  bool is_finished = false;
  bool line_lost = false;
  bool collision = false;
  bool clearance_violated = false;
  RaceStatus race_status = RaceStatus::kInProgress;
  InfractionCode infraction = InfractionCode::kNone;
  float sim_time_s = 0.0f;
  float cross_track_error_mm = 0.0f;
  float heading_error_deg = 0.0f;
  float min_obs_dist_mm = 9999.0f;
  uint8_t bypass_state = 0;
  RobotMode mode = RobotMode::kIdle;
};

struct FlightRecord {
  float timestamp_s = 0.0f;
  float x_mm = 0.0f;
  float y_mm = 0.0f;
  float theta_rad = 0.0f;
  float v_mps = 0.0f;
  float w_radps = 0.0f;
  float left_pwm = 0.0f;
  float right_pwm = 0.0f;
  float centroid_mm = 0.0f;
  std::array<float, LineSensorConfig::kSensorCount> ir_raw{};
  float tof_front_m = 0.0f;
  float tof_left_m = 0.0f;
  float tof_right_m = 0.0f;
  float cross_track_error_mm = 0.0f;
  uint8_t bypass_state = 0;
  bool line_lost = false;
  bool collision = false;
};

struct SimRunReport {
  bool success = false;
  std::string failure_reason;
  float lap_time_s = 0.0f;
  float total_distance_m = 0.0f;
  float max_cross_track_error_mm = 0.0f;
  float min_obstacle_clearance_mm = 9999.0f;
  float avg_speed_mps = 0.0f;
  float top_speed_mps = 0.0f;
  size_t waypoints_count = 0;
  FlightRecord crash_record{};
  std::vector<FlightRecord> pre_crash_tail;
  LapVerdict verdict{};
};

class SimEngine {
public:
  explicit SimEngine(const RobotConfig& config = RobotConfig{});
  ~SimEngine();

  SimEngine(const SimEngine&) = delete;
  SimEngine& operator=(const SimEngine&) = delete;
  SimEngine(SimEngine&&) noexcept = default;
  SimEngine& operator=(SimEngine&&) noexcept = default;

  bool load_track(const char* image_path);
  bool load_manifest(const TrackManifest& manifest);
  void set_obstacles(const std::vector<Obstacle>& obstacles);
  [[nodiscard]] const std::vector<Obstacle>& get_obstacles() const { return obstacles_; }
  std::vector<Obstacle>& get_obstacles_mut() { return obstacles_; }
  [[nodiscard]] const TrackManifest& get_manifest() const { return manifest_; }
  TrackManifest& get_manifest_mut() { return manifest_; }

  void reset_pose(RobotMode mode, bool clear_map = false);
  void reset_pose(RobotMode mode, bool clear_map, float start_x, float start_y, float start_theta);
  SimStepResult step(float dt = 0.002f);
  void update_from_telemetry(const TelemetryPacket& pkt, float dt, int32_t& prev_enc_l,
                             int32_t& prev_enc_r, bool& is_first_pkt);
  SimRunReport run_lap(RobotMode mode, float max_sim_time_s = 60.0f);

  void recompute_racing_trajectory();
  void recompute_racing_trajectory(float start_x, float start_y);
  [[nodiscard]] bool has_trajectory_data() const;

  [[nodiscard]] const SimRobotState& get_state() const { return state_; }
  [[nodiscard]] SimRobotState& get_state_mut() { return state_; }
  [[nodiscard]] const RobotCoordinator& get_robot() const { return robot_; }
  [[nodiscard]] RobotCoordinator& get_robot_mut() { return robot_; }
  [[nodiscard]] const DifferentialDriveKinematics& get_kinematics() const { return kinematics_; }
  [[nodiscard]] const RaceArbiter& get_arbiter() const { return arbiter_; }
  [[nodiscard]] RaceArbiter& get_arbiter_mut() { return arbiter_; }

  [[nodiscard]] const TofHitPoints& get_tof_hits() const { return hits_; }
  [[nodiscard]] Vector2 get_left_tof_origin() const { return l_origin_; }
  [[nodiscard]] Vector2 get_right_tof_origin() const { return r_origin_; }
  [[nodiscard]] float get_bar_center_x() const { return bar_center_x_; }
  [[nodiscard]] float get_bar_center_y() const { return bar_center_y_; }
  [[nodiscard]] const SensorData& get_sensor_input() const { return in_; }
  [[nodiscard]] float get_track_width_mm() const { return track_width_mm_; }
  [[nodiscard]] float get_track_height_mm() const { return track_height_mm_; }

  static void dump_post_mortem(std::ostream& os, const SimRunReport& report);
  static bool dump_crash_json(const char* filepath, const SimRunReport& report);

private:
  void sample_ir_sensors(float bar_center_x, float bar_center_y, float cos_th, float sin_th);
  void check_obstacle_collisions(SimStepResult& result);
  void record_flight_step();

  RobotCoordinator robot_;
  DifferentialDriveKinematics kinematics_;
  RaceArbiter arbiter_;
  SimRobotState state_{};
  std::vector<Obstacle> obstacles_;
  Image blurred_track_{};
  bool track_loaded_ = false;
  TrackManifest manifest_{};
  float track_width_mm_ = 0.0f;
  float track_height_mm_ = 0.0f;

  SensorData in_{};
  TofHitPoints hits_{};
  Vector2 l_origin_{};
  Vector2 r_origin_{};
  float bar_center_x_ = 0.0f;
  float bar_center_y_ = 0.0f;

  std::deque<FlightRecord> ring_buffer_;
  static constexpr size_t kRingBufferSize = 500;  // 1.0 giay o 500 Hz

  float min_obs_dist_mm_ = 9999.0f;
  float max_cte_mm_ = 0.0f;
  float sum_speed_mps_ = 0.0f;
  uint32_t speed_samples_count_ = 0;
  float top_speed_mps_ = 0.0f;
};

}  // namespace lf::sim
