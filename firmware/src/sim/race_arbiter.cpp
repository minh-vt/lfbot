#include "race_arbiter.hpp"

#include <algorithm>
#include <cmath>

namespace lf::sim {

namespace {

float cross_product_2d(float ax, float ay, float bx, float by) {
  return ax * by - ay * bx;
}

bool segments_intersect(float p1x, float p1y, float p2x, float p2y, float q1x, float q1y, float q2x,
                        float q2y) {
  const float d1 = cross_product_2d(p2x - p1x, p2y - p1y, q1x - p1x, q1y - p1y);
  const float d2 = cross_product_2d(p2x - p1x, p2y - p1y, q2x - p1x, q2y - p1y);
  const float d3 = cross_product_2d(q2x - q1x, q2y - q1y, p1x - q1x, p1y - q1y);
  const float d4 = cross_product_2d(q2x - q1x, q2y - q1y, p2x - q1x, p2y - q1y);

  return (((d1 > 0.0f && d2 < 0.0f) || (d1 < 0.0f && d2 > 0.0f)) &&
          ((d3 > 0.0f && d4 < 0.0f) || (d3 < 0.0f && d4 > 0.0f)));
}

}  // namespace

RaceArbiter::RaceArbiter() {
  setup_standard_fmaze_track();
  reset();
}

void RaceArbiter::reset() {
  verdict_ = LapVerdict{};
  verdict_.status = RaceStatus::kNotStarted;
  verdict_.total_sectors = gates_.size();
  current_sim_time_s_ = 0.0f;
  continuous_line_lost_timer_s_ = 0.0f;
  next_gate_idx_ = 0;
  prev_x_mm_ = 0.0f;
  prev_y_mm_ = 0.0f;
  is_initialized_ = false;
  entered_finish_box_ = false;
  finish_box_dwell_time_s_ = 0.0f;
}

void RaceArbiter::setup_standard_fmaze_track() {
  gates_.clear();

  // Sector Gate 1: Mid bottom straight heading East
  gates_.push_back({900.0f, 1100.0f, 900.0f, 1500.0f, 1.0f, 0.0f});

  // Sector Gate 2: East turn apex heading North/East
  gates_.push_back({1700.0f, 600.0f, 2100.0f, 1000.0f, 0.707f, -0.707f});

  // Sector Gate 3: Top straight heading West
  gates_.push_back({1200.0f, 100.0f, 1200.0f, 600.0f, -1.0f, 0.0f});

  // Sector Gate 4: West curve approach heading South
  gates_.push_back({200.0f, 850.0f, 600.0f, 850.0f, 0.0f, 1.0f});

  finish_box_ = FinishBoxGeometry{320.0f, 480.0f, 1220.0f, 1380.0f};
  verdict_.total_sectors = gates_.size();
}

void RaceArbiter::set_sector_gates(const std::vector<SectorGate>& gates) {
  gates_ = gates;
  verdict_.total_sectors = gates_.size();
}

void RaceArbiter::generate_gates_from_centerline(const std::vector<TrajectoryPoint>& centerline,
                                                 size_t num_gates) {
  if (centerline.size() < 10 || num_gates == 0) {
    return;
  }

  gates_.clear();
  const size_t n = centerline.size();
  const size_t step = n / (num_gates + 1);

  for (size_t g = 1; g <= num_gates; ++g) {
    const size_t idx = std::min(g * step, n - 2);
    const auto& pt = centerline.at(idx);

    const float cos_th = std::cos(pt.heading_rad);
    const float sin_th = std::sin(pt.heading_rad);

    // Lateral normal across track width (+/- 250mm)
    constexpr float kHalfWidthMm = 250.0f;
    const float x1 = pt.x_mm - kHalfWidthMm * sin_th;
    const float y1 = pt.y_mm + kHalfWidthMm * cos_th;
    const float x2 = pt.x_mm + kHalfWidthMm * sin_th;
    const float y2 = pt.y_mm - kHalfWidthMm * cos_th;

    gates_.push_back({x1, y1, x2, y2, cos_th, sin_th});
  }
  verdict_.total_sectors = gates_.size();
}

void RaceArbiter::set_finish_box(const FinishBoxGeometry& box) {
  finish_box_ = box;
}

void RaceArbiter::check_arena_boundaries(const ArbiterGroundTruth& gt) {
  if (gt.x_mm < arena_min_x_ || gt.x_mm > arena_max_x_ || gt.y_mm < arena_min_y_ ||
      gt.y_mm > arena_max_y_) {
    verdict_.status = RaceStatus::kDisqualified;
    verdict_.infraction = InfractionCode::kOutOfBounds;
    verdict_.infraction_detail = "Vehicle breached arena boundary limits";
  }
}

void RaceArbiter::check_line_loss_timer(const ArbiterGroundTruth& gt, float dt) {
  if (!gt.line_detected) {
    continuous_line_lost_timer_s_ += dt;
    if (continuous_line_lost_timer_s_ > 5.0f) {
      verdict_.status = RaceStatus::kDisqualified;
      verdict_.infraction = InfractionCode::kLineLostTimeout;
      verdict_.infraction_detail = "Off-line duration exceeded 5.0s (Rule 5.5)";
    }
  } else {
    continuous_line_lost_timer_s_ = 0.0f;
  }
  verdict_.continuous_off_line_s = continuous_line_lost_timer_s_;
}

void RaceArbiter::check_sector_gates(const ArbiterGroundTruth& gt) {
  if (!is_initialized_) {
    return;
  }

  const float vx = gt.x_mm - prev_x_mm_;
  const float vy = gt.y_mm - prev_y_mm_;
  const float move_dist_sq = vx * vx + vy * vy;
  if (move_dist_sq < 1e-4f) {
    return;
  }

  // Check all gates for illegal out-of-order or reverse crossings
  for (size_t i = 0; i < gates_.size(); ++i) {
    const auto& gate = gates_.at(i);
    if (!segments_intersect(prev_x_mm_, prev_y_mm_, gt.x_mm, gt.y_mm, gate.x1_mm, gate.y1_mm,
                            gate.x2_mm, gate.y2_mm)) {
      continue;
    }

    const float dot = vx * gate.normal_x + vy * gate.normal_y;
    if (dot < 0.0f) {
      verdict_.status = RaceStatus::kDisqualified;
      verdict_.infraction = InfractionCode::kWrongWay;
      verdict_.infraction_detail = "Vehicle crossed gate in reverse direction (Rule Violation)";
      return;
    }

    if (i == next_gate_idx_) {
      next_gate_idx_++;
      verdict_.sectors_cleared = next_gate_idx_;
    } else if (i > next_gate_idx_) {
      verdict_.status = RaceStatus::kDisqualified;
      verdict_.infraction = InfractionCode::kGateSkipped;
      verdict_.infraction_detail = "Vehicle skipped sector gate without clearance";
      return;
    }
  }
}

void RaceArbiter::check_finish_condition(const ArbiterGroundTruth& gt) {
  // Can only trigger finish condition after ALL sector gates have been legally cleared
  if (next_gate_idx_ < gates_.size()) {
    return;
  }

  if (finish_box_.contains(gt.x_mm, gt.y_mm)) {
    entered_finish_box_ = true;

    // Must come to a controlled halt inside the finish box (Rule 4.6 & SPEC)
    if (gt.v_mps <= 0.15f || gt.active_brake) {
      // Guard against physics anomaly: On 6m track, average speed cannot violate mu*g envelope
      if (current_sim_time_s_ < 2.0f) {
        verdict_.status = RaceStatus::kDisqualified;
        verdict_.infraction = InfractionCode::kPhysicsAnomaly;
        verdict_.infraction_detail = "Lap completed in impossibly short physical time (< 2.0s)";
        return;
      }

      verdict_.status = RaceStatus::kCompleted;
      verdict_.stopped_in_finish_box = true;
      verdict_.official_lap_time_s = current_sim_time_s_;
    }
  }
}

void RaceArbiter::step(const ArbiterGroundTruth& gt, float dt) {
  if (is_race_over()) {
    return;
  }

  if (!is_initialized_) {
    prev_x_mm_ = gt.x_mm;
    prev_y_mm_ = gt.y_mm;
    is_initialized_ = true;
    verdict_.status = RaceStatus::kInProgress;
  }

  current_sim_time_s_ += dt;
  const float dx = (gt.x_mm - prev_x_mm_) * 0.001f;
  const float dy = (gt.y_mm - prev_y_mm_) * 0.001f;
  verdict_.total_distance_m += std::sqrt(dx * dx + dy * dy);

  // 1. Check physical collisions (SAT OBB / Clearance)
  if (gt.physical_collision || gt.min_obs_dist_mm < 129.5f) {
    verdict_.status = RaceStatus::kDisqualified;
    verdict_.infraction = InfractionCode::kObstacleCollision;
    verdict_.infraction_detail = "Physical collision with obstacle detected (Contact < 129.5mm)";
    return;
  }

  // 2. Check arena limits
  check_arena_boundaries(gt);
  if (is_race_over()) {
    return;
  }

  // 3. Check 5s off-line timeout (Rule 5.5)
  check_line_loss_timer(gt, dt);
  if (is_race_over()) {
    return;
  }

  // 4. Check 3-minute time limit (Rule 5.2)
  if (current_sim_time_s_ > max_time_limit_s_) {
    verdict_.status = RaceStatus::kDisqualified;
    verdict_.infraction = InfractionCode::kTimeLimitExceeded;
    verdict_.infraction_detail = "Time limit exceeded (> 180s per Rule 5.2)";
    return;
  }

  // 5. Check Sector Gates progression
  check_sector_gates(gt);
  if (is_race_over()) {
    return;
  }

  // 6. Check Finish Box arrival & dwell
  check_finish_condition(gt);

  prev_x_mm_ = gt.x_mm;
  prev_y_mm_ = gt.y_mm;
}

const char* RaceArbiter::infraction_to_string(InfractionCode code) {
  switch (code) {
    case InfractionCode::kNone:
      return "NONE";
    case InfractionCode::kObstacleCollision:
      return "OBSTACLE_COLLISION";
    case InfractionCode::kOutOfBounds:
      return "OUT_OF_BOUNDS";
    case InfractionCode::kLineLostTimeout:
      return "LINE_LOST_5S_TIMEOUT";
    case InfractionCode::kWrongWay:
      return "WRONG_WAY";
    case InfractionCode::kGateSkipped:
      return "GATE_SKIPPED";
    case InfractionCode::kTimeLimitExceeded:
      return "TIME_LIMIT_EXCEEDED";
    case InfractionCode::kPhysicsAnomaly:
      return "PHYSICS_ANOMALY";
    default:
      return "UNKNOWN_INFRACTION";
  }
}

const char* RaceArbiter::status_to_string(RaceStatus status) {
  switch (status) {
    case RaceStatus::kNotStarted:
      return "NOT_STARTED";
    case RaceStatus::kInProgress:
      return "IN_PROGRESS";
    case RaceStatus::kCompleted:
      return "COMPLETED";
    case RaceStatus::kDisqualified:
      return "DISQUALIFIED";
    default:
      return "UNKNOWN_STATUS";
  }
}

}  // namespace lf::sim
