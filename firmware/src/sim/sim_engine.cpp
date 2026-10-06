#include "sim_engine.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "config/constants.hpp"
#include "track/trajectory_planner.hpp"

namespace lf::sim {

namespace {

constexpr std::array<std::array<float, 3>, 3> kKernelWeights = {
  {{{0.06f, 0.14f, 0.06f}}, {{0.14f, 0.20f, 0.14f}}, {{0.06f, 0.14f, 0.06f}}}};

}  // namespace

SimEngine::SimEngine(const RobotConfig& config)
  : robot_(config), kinematics_(config.kinematics.wheel_base_m) {
  robot_.init();
}

SimEngine::~SimEngine() {
  if (track_loaded_ && blurred_track_.data != nullptr) {
    UnloadImage(blurred_track_);
    blurred_track_ = Image{};
    track_loaded_ = false;
  }
}

bool SimEngine::load_track(const char* image_path) {
  if (track_loaded_ && blurred_track_.data != nullptr) {
    UnloadImage(blurred_track_);
    blurred_track_ = Image{};
    track_loaded_ = false;
  }

  Image raw = LoadImage(image_path);
  if (raw.data == nullptr) {
    return false;
  }

  ImageFormat(&raw, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);
  blurred_track_ = ImageCopy(raw);
  ImageBlurGaussian(&blurred_track_, 2);
  UnloadImage(raw);

  track_loaded_ = true;
  return true;
}

void SimEngine::set_obstacles(const std::vector<Obstacle>& obstacles) {
  obstacles_ = obstacles;
}

void SimEngine::reset_pose(RobotMode mode, bool clear_map) {
  state_.x = kStartPosX;
  state_.y = kStartPosY;
  state_.theta = 0.0f;
  state_.v = 0.0f;
  state_.w = 0.0f;
  state_.left_pwm = 0.0f;
  state_.right_pwm = 0.0f;
  state_.enc_l = 0;
  state_.enc_r = 0;
  state_.is_paused = false;
  state_.sim_time = 0.0f;
  state_.trail.clear();
  state_.is_crashed = false;
  state_.crashed_obstacle_idx = -1;
  state_.wheel_dist_l_m = 0.0;
  state_.wheel_dist_r_m = 0.0;
  arbiter_.reset();

  min_obs_dist_mm_ = 9999.0f;
  max_cte_mm_ = 0.0f;
  sum_speed_mps_ = 0.0f;
  speed_samples_count_ = 0;
  top_speed_mps_ = 0.0f;
  ring_buffer_.clear();

  if (clear_map) {
    robot_.init();
    state_.unoptimized_trail.clear();
    state_.planned_trajectory.clear();
    state_.has_run1_data = false;
    state_.has_run2_data = false;
    state_.run1_time_s = 0.0f;
    state_.run2_time_s = 0.0f;
  } else {
    robot_.reset_pose();
  }

  robot_.set_mode(mode);
}

void SimEngine::recompute_racing_trajectory() {
  if (state_.unoptimized_trail.size() < 10) {
    return;
  }

  std::vector<TrajectoryPoint> raw_path;
  raw_path.reserve(state_.unoptimized_trail.size());
  for (const auto& pt : state_.unoptimized_trail) {
    raw_path.push_back({pt.x, pt.y, robot_.get_config().profiler.safe_map_speed_mps, 0.0f});
  }

  std::vector<ObstacleGeometry> obs_geom;
  obs_geom.reserve(obstacles_.size());
  for (const auto& obs : obstacles_) {
    obs_geom.push_back({obs.pos.x, obs.pos.y, obs.width_mm, obs.height_mm, obs.angle_deg});
  }

  const TrajectoryPlanner planner(robot_.get_config().environment, robot_.get_config().profiler);
  state_.planned_trajectory = planner.generate_racing_line(raw_path, obs_geom);

  // Chuyen toa do arena frame sang robot local odometry frame
  std::vector<TrajectoryPoint> local_traj = state_.planned_trajectory;
  for (auto& pt : local_traj) {
    pt.x_mm -= kStartPosX;
    pt.y_mm -= kStartPosY;
  }
  robot_.set_racing_trajectory(local_traj);
  arbiter_.generate_gates_from_centerline(state_.planned_trajectory, 4);
}

void SimEngine::sample_ir_sensors(float bar_center_x, float bar_center_y, float cos_th,
                                  float sin_th) {
  if (!track_loaded_ || blurred_track_.data == nullptr) {
    return;
  }

  for (size_t i = 0; i < LineSensorConfig::kSensorCount; ++i) {
    const float offset_mm = robot_.get_config().line_sensor.weights_mm.at(i);
    const float sx = bar_center_x - offset_mm * sin_th;
    const float sy = bar_center_y + offset_mm * cos_th;

    // 3x3 Area integration
    float total_reflectivity = 0.0f;
    for (size_t r = 0; r < 3; ++r) {
      const float dy = (static_cast<float>(r) - 1.0f) * 1.5f;
      for (size_t c = 0; c < 3; ++c) {
        const float dx = (static_cast<float>(c) - 1.0f) * 1.5f;
        const float kx = sx + dx;
        const float ky = sy + dy;
        const int px = std::clamp(static_cast<int>(kx), 0, blurred_track_.width - 1);
        const int py = std::clamp(static_cast<int>(ky), 0, blurred_track_.height - 1);

        const Color pixel = GetImageColor(blurred_track_, px, py);
        const float brightness = static_cast<float>(pixel.r + pixel.g + pixel.b) / (3.0f * 255.0f);
        total_reflectivity += (1.0f - brightness) * kKernelWeights.at(r).at(c);
      }
    }
    in_.ir_raw.at(i) = total_reflectivity;
  }
}

void SimEngine::check_obstacle_collisions(SimStepResult& result) {
  const float clearance_radius = robot_.get_config().environment.clearance_radius_mm();
  constexpr float kPhysicalContactRadius = 129.5f;  // R_obs (60.5) + W_robot / 2 (69.0)

  float min_dist = 9999.0f;
  int nearest_idx = -1;

  for (size_t i = 0; i < obstacles_.size(); ++i) {
    const auto& obs = obstacles_.at(i);
    const float dx = state_.x - obs.pos.x;
    const float dy = state_.y - obs.pos.y;
    const float dist = std::sqrt(dx * dx + dy * dy);

    if (dist < min_dist) {
      min_dist = dist;
      nearest_idx = static_cast<int>(i);
    }
  }

  result.min_obs_dist_mm = min_dist;
  min_obs_dist_mm_ = std::min(min_dist, min_obs_dist_mm_);

  if (min_dist < kPhysicalContactRadius) {
    result.collision = true;
    state_.is_crashed = true;
    state_.crashed_obstacle_idx = nearest_idx;
  } else if (min_dist < clearance_radius) {
    result.clearance_violated = true;
  }
}

void SimEngine::record_flight_step() {
  FlightRecord rec{};
  rec.timestamp_s = state_.sim_time;
  rec.x_mm = state_.x;
  rec.y_mm = state_.y;
  rec.theta_rad = state_.theta;
  rec.v_mps = state_.v;
  rec.w_radps = state_.w;
  rec.left_pwm = state_.left_pwm;
  rec.right_pwm = state_.right_pwm;
  rec.centroid_mm = robot_.get_telemetry().centroid_pos_mm;
  rec.ir_raw = in_.ir_raw;
  rec.tof_front_m = in_.tof_front_m;
  rec.tof_left_m = in_.tof_left_m;
  rec.tof_right_m = in_.tof_right_m;
  rec.cross_track_error_mm = robot_.get_telemetry().cross_track_error_m * 1000.0f;
  rec.bypass_state = robot_.get_telemetry().bypass_state;
  rec.line_lost = robot_.get_telemetry().line_lost;
  rec.collision = state_.is_crashed;

  ring_buffer_.push_back(rec);
  if (ring_buffer_.size() > kRingBufferSize) {
    ring_buffer_.pop_front();
  }
}

SimStepResult SimEngine::step(float dt) {
  SimStepResult result{};
  result.mode = robot_.get_mode();

  if (state_.is_paused) {
    result.sim_time_s = state_.sim_time;
    return result;
  }

  in_.dt = dt;
  const float cos_th = std::cos(state_.theta);
  const float sin_th = std::sin(state_.theta);

  const float lookahead_mm = robot_.get_config().kinematics.sensor_lookahead_m * 1000.0f;
  bar_center_x_ = state_.x + lookahead_mm * cos_th;
  bar_center_y_ = state_.y + lookahead_mm * sin_th;

  // 1. Doc gia tri quang hoc 8 mat
  sample_ir_sensors(bar_center_x_, bar_center_y_, cos_th, sin_th);

  // 2. Raycast Laser ToF 3 truc
  raycast_all_tof(state_.x, state_.y, state_.theta, bar_center_x_, bar_center_y_, obstacles_, in_,
                  hits_, l_origin_, r_origin_, robot_.get_config().obstacle.front_detect_dist_m,
                  0.20f);

  in_.enc_left_ticks = state_.enc_l;
  in_.enc_right_ticks = state_.enc_r;

  // 3. Cap nhat bo dieu phoi robot
  const RobotMode prev_mode = robot_.get_mode();
  MotorCmd out{};
  robot_.update(in_, out);

  state_.sim_time += dt;

  // 4. Kiem tra va cham vat can
  check_obstacle_collisions(result);

  state_.left_pwm = out.left_pwm;
  state_.right_pwm = out.right_pwm;

  const float max_speed = robot_.get_config().profiler.max_straight_speed_mps;
  float v_l = out.left_pwm * max_speed;
  float v_r = out.right_pwm * max_speed;

  if (out.active_brake) {
    v_l = 0.0f;
    v_r = 0.0f;
  }

  // 5. Tinh toan dong hoc vi sai kinematics
  const auto twist = kinematics_.forward(v_l, v_r);
  state_.v = twist.linear_v_mps;
  state_.w = twist.angular_w_radps;
  state_.lateral_accel_mps2 = state_.v * state_.w;
  state_.longitudinal_accel_mps2 = (dt > 1e-4f) ? ((state_.v - state_.prev_v) / dt) : 0.0f;
  state_.prev_v = state_.v;

  sum_speed_mps_ += state_.v;
  speed_samples_count_++;
  top_speed_mps_ = std::max(state_.v, top_speed_mps_);

  const float cte_mm = std::abs(robot_.get_telemetry().cross_track_error_m * 1000.0f);
  max_cte_mm_ = std::max(cte_mm, max_cte_mm_);

  state_.wheel_dist_l_m += v_l * dt;
  state_.wheel_dist_r_m += v_r * dt;
  const float m_per_tick = robot_.get_config().kinematics.meters_per_tick();
  state_.enc_l = static_cast<int32_t>(std::round(state_.wheel_dist_l_m / m_per_tick));
  state_.enc_r = static_cast<int32_t>(std::round(state_.wheel_dist_r_m / m_per_tick));

  const float mid_th = state_.theta + 0.5f * state_.w * dt;
  state_.x += state_.v * std::cos(mid_th) * dt * 1000.0f;
  state_.y += state_.v * std::sin(mid_th) * dt * 1000.0f;
  state_.theta += state_.w * dt;

  // 6. Ghi vet quy dao robot
  const float d_sq = state_.trail.empty()
                       ? 100.0f
                       : (state_.trail.back().x - state_.x) * (state_.trail.back().x - state_.x) +
                           (state_.trail.back().y - state_.y) * (state_.trail.back().y - state_.y);
  if (state_.trail.empty() || d_sq > 64.0f) {
    state_.trail.push_back(Vector2{state_.x, state_.y});
    if (state_.trail.size() > kMaxTrailPoints) {
      state_.trail.pop_front();
    }
  }

  // 7. Ghi hop den Flight Recorder
  record_flight_step();

  // 8. Phat hien hoan thanh va vi pham qua RaceArbiter
  ArbiterGroundTruth gt{};
  gt.x_mm = state_.x;
  gt.y_mm = state_.y;
  gt.heading_rad = state_.theta;
  gt.v_mps = state_.v;
  gt.w_radps = state_.w;
  gt.line_detected = !robot_.get_telemetry().line_lost;
  gt.min_obs_dist_mm = result.min_obs_dist_mm;
  gt.physical_collision = state_.is_crashed;
  gt.active_brake = out.active_brake;

  arbiter_.step(gt, dt);

  const auto& verdict = arbiter_.get_verdict();
  result.race_status = verdict.status;
  result.infraction = verdict.infraction;

  if (verdict.status == RaceStatus::kCompleted) {
    if (prev_mode == RobotMode::kRun1Map) {
      state_.run1_time_s = verdict.official_lap_time_s;
      state_.has_run1_data = true;
      if (state_.trail.size() > 10) {
        state_.unoptimized_trail.assign(state_.trail.begin(), state_.trail.end());
        recompute_racing_trajectory();
      }
    } else if (prev_mode == RobotMode::kRun2Sprint) {
      state_.run2_time_s = verdict.official_lap_time_s;
      state_.has_run2_data = true;
    }
    result.is_finished = true;
  } else if (verdict.status == RaceStatus::kDisqualified) {
    result.is_finished = true;
  }

  result.sim_time_s = state_.sim_time;
  result.line_lost = robot_.get_telemetry().line_lost;
  result.cross_track_error_mm = cte_mm;
  result.heading_error_deg = robot_.get_telemetry().heading_error_rad * (180.0f / constants::kPi);
  result.bypass_state = robot_.get_telemetry().bypass_state;
  result.mode = robot_.get_mode();

  return result;
}

SimRunReport SimEngine::run_lap(RobotMode mode, float max_sim_time_s) {
  SimRunReport report{};
  reset_pose(mode, false);

  if (mode == RobotMode::kRun2Sprint) {
    if (state_.planned_trajectory.empty() && !state_.unoptimized_trail.empty()) {
      recompute_racing_trajectory();
    } else if (!state_.planned_trajectory.empty()) {
      std::vector<TrajectoryPoint> local_traj = state_.planned_trajectory;
      for (auto& pt : local_traj) {
        pt.x_mm -= kStartPosX;
        pt.y_mm -= kStartPosY;
      }
      robot_.set_racing_trajectory(local_traj);
    }
  }

  constexpr float kDt = 0.002f;  // 500 Hz

  while (state_.sim_time < max_sim_time_s) {
    const SimStepResult step_res = step(kDt);

    if (step_res.is_finished) {
      break;
    }
  }

  const auto& verdict = arbiter_.get_verdict();
  report.verdict = verdict;
  if (verdict.status == RaceStatus::kCompleted) {
    report.success = true;
    report.lap_time_s = verdict.official_lap_time_s;
  } else {
    report.success = false;
    if (verdict.infraction != InfractionCode::kNone) {
      report.failure_reason = std::string(RaceArbiter::infraction_to_string(verdict.infraction)) +
                              ": " + verdict.infraction_detail;
    } else {
      report.failure_reason = "TIMEOUT: Lap exceeded maximum simulation time limit";
    }
    report.crash_record = ring_buffer_.empty() ? FlightRecord{} : ring_buffer_.back();
    report.pre_crash_tail.assign(ring_buffer_.begin(), ring_buffer_.end());
  }

  report.total_distance_m = robot_.get_telemetry().total_distance_m;
  report.max_cross_track_error_mm = max_cte_mm_;
  report.min_obstacle_clearance_mm = min_obs_dist_mm_;
  report.avg_speed_mps =
    (speed_samples_count_ > 0) ? (sum_speed_mps_ / static_cast<float>(speed_samples_count_)) : 0.0f;
  report.top_speed_mps = top_speed_mps_;
  report.waypoints_count = state_.trail.size();

  return report;
}

void SimEngine::dump_post_mortem(std::ostream& os, const SimRunReport& report) const {
  os << "\n======================= [SIMULATION FAILURE POST-MORTEM] =======================\n";
  os << "Failure Reason    : " << report.failure_reason << "\n";
  os << "Official Status   : " << RaceArbiter::status_to_string(report.verdict.status) << "\n";
  os << "Sectors Cleared   : " << report.verdict.sectors_cleared << " / "
     << report.verdict.total_sectors << "\n";
  os << "Finish Box Halt   : " << (report.verdict.stopped_in_finish_box ? "YES" : "NO") << "\n";
  os << "Timestamp & Step  : t = " << std::fixed << std::setprecision(3)
     << report.crash_record.timestamp_s << " s | Step #"
     << static_cast<uint32_t>(report.crash_record.timestamp_s / 0.002f) << "\n";
  os << "Arena Coordinates : X = " << std::fixed << std::setprecision(1) << report.crash_record.x_mm
     << " mm, Y = " << report.crash_record.y_mm
     << " mm, Heading = " << (report.crash_record.theta_rad * 180.0f / constants::kPi) << " deg\n";
  os << "Kinematic State   : v = " << std::fixed << std::setprecision(2)
     << report.crash_record.v_mps << " m/s, w = " << report.crash_record.w_radps << " rad/s\n";
  os << "Actuator Output   : Left PWM = " << std::fixed << std::setprecision(2)
     << report.crash_record.left_pwm << ", Right PWM = " << report.crash_record.right_pwm << "\n";
  os << "Sensors           : Centroid = " << std::fixed << std::setprecision(1)
     << report.crash_record.centroid_mm << " mm, ToF Front = " << std::fixed << std::setprecision(2)
     << report.crash_record.tof_front_m << " m, Left = " << report.crash_record.tof_left_m
     << " m, Right = " << report.crash_record.tof_right_m << " m\n";
  os << "Tracking Error    : Cross-Track Error (CTE) = " << std::fixed << std::setprecision(1)
     << report.crash_record.cross_track_error_mm << " mm\n";
  os << "Min Obstacle Dist : " << std::fixed << std::setprecision(1)
     << report.min_obstacle_clearance_mm << " mm\n";

  if (!report.pre_crash_tail.empty()) {
    os << "\n--- Pre-Crash Telemetry History (Last 10 steps) ---\n";
    os << "Step   | Time (s) | X (mm)   | Y (mm)   | Heading  | v (m/s) | Centroid | LineLost | "
          "ToF_F (m)\n";
    const size_t start_idx =
      (report.pre_crash_tail.size() > 10) ? (report.pre_crash_tail.size() - 10) : 0;
    for (size_t i = start_idx; i < report.pre_crash_tail.size(); ++i) {
      const auto& s = report.pre_crash_tail.at(i);
      os << std::setw(6) << i << " | " << std::fixed << std::setprecision(3) << std::setw(8)
         << s.timestamp_s << " | " << std::fixed << std::setprecision(1) << std::setw(8) << s.x_mm
         << " | " << std::setw(8) << s.y_mm << " | " << std::setw(8)
         << (s.theta_rad * 180.0f / constants::kPi) << " | " << std::fixed << std::setprecision(2)
         << std::setw(7) << s.v_mps << " | " << std::fixed << std::setprecision(1) << std::setw(8)
         << s.centroid_mm << " | " << std::setw(8) << (s.line_lost ? "LOST" : "OK") << " | "
         << std::fixed << std::setprecision(2) << std::setw(8) << s.tof_front_m << "\n";
    }
  }
  os << "================================================================================\n\n";
}

bool SimEngine::dump_crash_json(const char* filepath, const SimRunReport& report) const {
  std::ofstream fs(filepath);
  if (!fs.is_open()) {
    return false;
  }

  fs << "{\n";
  fs << "  \"success\": " << (report.success ? "true" : "false") << ",\n";
  fs << "  \"failure_reason\": \"" << report.failure_reason << "\",\n";
  fs << "  \"lap_time_s\": " << report.lap_time_s << ",\n";
  fs << "  \"crash_record\": {\n";
  fs << "    \"time_s\": " << report.crash_record.timestamp_s << ",\n";
  fs << "    \"x_mm\": " << report.crash_record.x_mm << ",\n";
  fs << "    \"y_mm\": " << report.crash_record.y_mm << ",\n";
  fs << "    \"theta_rad\": " << report.crash_record.theta_rad << ",\n";
  fs << "    \"v_mps\": " << report.crash_record.v_mps << ",\n";
  fs << "    \"w_radps\": " << report.crash_record.w_radps << ",\n";
  fs << "    \"left_pwm\": " << report.crash_record.left_pwm << ",\n";
  fs << "    \"right_pwm\": " << report.crash_record.right_pwm << ",\n";
  fs << "    \"centroid_mm\": " << report.crash_record.centroid_mm << ",\n";
  fs << "    \"tof_front_m\": " << report.crash_record.tof_front_m << ",\n";
  fs << "    \"tof_left_m\": " << report.crash_record.tof_left_m << ",\n";
  fs << "    \"tof_right_m\": " << report.crash_record.tof_right_m << "\n";
  fs << "  },\n";
  fs << "  \"pre_crash_steps_count\": " << report.pre_crash_tail.size() << "\n";
  fs << "}\n";

  return true;
}

}  // namespace lf::sim
