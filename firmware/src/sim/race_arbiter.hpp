#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "config/types.hpp"

namespace lf::sim {

enum class RaceStatus : uint8_t {
  kNotStarted,
  kInProgress,
  kCompleted,
  kDisqualified,
};

enum class InfractionCode : uint8_t {
  kNone,
  kObstacleCollision,
  kOutOfBounds,
  kLineLostTimeout,
  kWrongWay,
  kGateSkipped,
  kTimeLimitExceeded,
  kPhysicsAnomaly,
};

struct SectorGate {
  float x1_mm = 0.0f;
  float y1_mm = 0.0f;
  float x2_mm = 0.0f;
  float y2_mm = 0.0f;
  float normal_x = 0.0f;
  float normal_y = 0.0f;
};

struct FinishBoxGeometry {
  float min_x_mm = 0.0f;
  float max_x_mm = 0.0f;
  float min_y_mm = 0.0f;
  float max_y_mm = 0.0f;

  [[nodiscard]] bool contains(float x_mm, float y_mm) const {
    return x_mm >= min_x_mm && x_mm <= max_x_mm && y_mm >= min_y_mm && y_mm <= max_y_mm;
  }
};

struct ArbiterGroundTruth {
  float x_mm = 0.0f;
  float y_mm = 0.0f;
  float heading_rad = 0.0f;
  float v_mps = 0.0f;
  float w_radps = 0.0f;
  bool line_detected = true;
  float min_obs_dist_mm = 9999.0f;
  bool physical_collision = false;
  bool active_brake = false;
};

struct LapVerdict {
  RaceStatus status = RaceStatus::kNotStarted;
  InfractionCode infraction = InfractionCode::kNone;
  float official_lap_time_s = 0.0f;
  float continuous_off_line_s = 0.0f;
  float total_distance_m = 0.0f;
  size_t sectors_cleared = 0;
  size_t total_sectors = 0;
  bool stopped_in_finish_box = false;
  std::string infraction_detail;
};

class RaceArbiter {
public:
  RaceArbiter();

  void reset();
  void setup_standard_fmaze_track();
  void set_sector_gates(const std::vector<SectorGate>& gates);
  void generate_gates_from_centerline(const std::vector<TrajectoryPoint>& centerline,
                                      size_t num_gates = 3);
  void set_finish_box(const FinishBoxGeometry& box);
  void set_arena_boundaries(float min_x, float max_x, float min_y, float max_y) {
    arena_min_x_ = min_x;
    arena_max_x_ = max_x;
    arena_min_y_ = min_y;
    arena_max_y_ = max_y;
  }
  void set_max_lap_time(float max_time_s) { max_time_limit_s_ = max_time_s; }

  void step(const ArbiterGroundTruth& gt, float dt);

  [[nodiscard]] const LapVerdict& get_verdict() const { return verdict_; }
  [[nodiscard]] bool is_race_over() const {
    return verdict_.status == RaceStatus::kCompleted ||
           verdict_.status == RaceStatus::kDisqualified;
  }

  [[nodiscard]] static const char* infraction_to_string(InfractionCode code);
  [[nodiscard]] static const char* status_to_string(RaceStatus status);

  [[nodiscard]] float get_off_line_timer_s() const { return continuous_line_lost_timer_s_; }
  [[nodiscard]] size_t get_next_gate_idx() const { return next_gate_idx_; }
  [[nodiscard]] size_t get_total_gates() const { return gates_.size(); }

private:
  void check_arena_boundaries(const ArbiterGroundTruth& gt);
  void check_line_loss_timer(const ArbiterGroundTruth& gt, float dt);
  void check_sector_gates(const ArbiterGroundTruth& gt);
  void check_finish_condition(const ArbiterGroundTruth& gt);

  std::vector<SectorGate> gates_;
  FinishBoxGeometry finish_box_{};
  LapVerdict verdict_{};

  float current_sim_time_s_ = 0.0f;
  float continuous_line_lost_timer_s_ = 0.0f;
  float max_time_limit_s_ = 180.0f;  // Official Rule 5.2: 3 minutes max
  float arena_min_x_ = -50.0f;
  float arena_max_x_ = 2450.0f;
  float arena_min_y_ = -50.0f;
  float arena_max_y_ = 1650.0f;
  size_t next_gate_idx_ = 0;
  float prev_x_mm_ = 0.0f;
  float prev_y_mm_ = 0.0f;
  bool is_initialized_ = false;
  bool entered_finish_box_ = false;
  float finish_box_dwell_time_s_ = 0.0f;
};

}  // namespace lf::sim
