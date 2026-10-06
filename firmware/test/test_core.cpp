#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "config/constants.hpp"
#include "config/robot_config.hpp"
#include "control/relay_tuner.hpp"
#include "control/stanley_tracker.hpp"
#include "control/trajectory_follower.hpp"
#include "math/collision.hpp"
#include "math/kinematics.hpp"
#include "system/robot_coordinator.hpp"
#include "system/telemetry_packet.hpp"
#include "track/global_trajectory_optimizer.hpp"
#include "track/trajectory_planner.hpp"

namespace {

using lf::constants::kPi;

void test_centroid_calculation() {
  const lf::RobotConfig config{};
  lf::RobotCoordinator robot(config);
  robot.init();

  lf::SensorData in{};
  in.dt = 0.002f;

  // Case 1: Vạch đen ngay giữa 2 mắt tâm (mắt 3: -4.5mm và mắt 4: +4.5mm)
  for (size_t i = 0; i < lf::LineSensorConfig::kSensorCount; ++i) {
    in.ir_raw.at(i) = 0.0f;
  }
  in.ir_raw.at(3) = 1.0f;
  in.ir_raw.at(4) = 1.0f;

  lf::MotorCmd out{};
  robot.update(in, out);

  float pos = robot.get_telemetry().centroid_pos_mm;
  std::cout << "[TEST] Centroid tai tam vach: " << pos << " mm\n";
  assert(std::abs(pos) < 0.1f);

  // Case 2: Vạch đen lệch sang trái ngoài cùng (mắt 0 đọc 1.0f)
  for (size_t i = 0; i < lf::LineSensorConfig::kSensorCount; ++i) {
    in.ir_raw.at(i) = 0.0f;
  }
  in.ir_raw.at(0) = 1.0f;
  robot.update(in, out);
  pos = robot.get_telemetry().centroid_pos_mm;
  std::cout << "[TEST] Centroid lech trai ngoai cung: " << pos << " mm\n";
  assert(pos < -30.0f);
}

void test_steering_direction() {
  const lf::RobotConfig config{};
  lf::RobotCoordinator robot(config);
  robot.init();
  robot.set_mode(lf::RobotMode::kRun1Map);

  lf::SensorData in{};
  in.dt = 0.002f;
  in.tof_front_m = 1.0f;

  // Case 1: Line on the LEFT (sensor 0 = 1.0, centroid < 0)
  for (size_t i = 0; i < lf::LineSensorConfig::kSensorCount; ++i) {
    in.ir_raw.at(i) = 0.0f;
  }
  in.ir_raw.at(0) = 1.0f;
  lf::MotorCmd out{};
  robot.update(in, out);

  std::cout << "[TEST] Line on Left -> Left PWM: " << out.left_pwm
            << ", Right PWM: " << out.right_pwm << "\n";
  // Left wheel must be SLOWER than Right wheel to turn LEFT towards the line
  assert(out.left_pwm < out.right_pwm);

  // Case 2: Line on the RIGHT (sensor 7 = 1.0, centroid > 0)
  robot.init();
  robot.set_mode(lf::RobotMode::kRun1Map);
  for (size_t i = 0; i < lf::LineSensorConfig::kSensorCount; ++i) {
    in.ir_raw.at(i) = 0.0f;
  }
  in.ir_raw.at(7) = 1.0f;
  robot.update(in, out);

  std::cout << "[TEST] Line on Right -> Left PWM: " << out.left_pwm
            << ", Right PWM: " << out.right_pwm << "\n";
  // Left wheel must be FASTER than Right wheel to turn RIGHT towards the line
  assert(out.left_pwm > out.right_pwm);
}

std::vector<lf::TrajectoryPoint> generate_test_curve_path() {
  std::vector<lf::TrajectoryPoint> raw_path;
  for (int x = 0; x <= 1500; x += 10) {
    raw_path.push_back({static_cast<float>(x), 0.0f, 1.0f, 0.0f});
  }
  constexpr int kCurveSteps = 30;
  for (int i = 1; i <= kCurveSteps; ++i) {
    const float th = -0.5f * kPi + (0.5f * kPi) * (static_cast<float>(i) / kCurveSteps);
    const float cx = 1500.0f + 200.0f * std::cos(th);
    const float cy = 200.0f + 200.0f * std::sin(th);
    raw_path.push_back({cx, cy, 1.0f, 0.0f});
  }
  for (int y = 210; y <= 1200; y += 10) {
    raw_path.push_back({1700.0f, static_cast<float>(y), 1.0f, 0.0f});
  }
  return raw_path;
}

void verify_trajectory_speed_profiling(const std::vector<lf::TrajectoryPoint>& racing_line,
                                       const lf::ProfilerConfig& profiler_cfg) {
  assert(!racing_line.empty());

  // Tra cứu tại s ~ 0.5m: Đang tăng tốc trên đoạn thẳng (x ~ 500, idx ~ 50)
  const float v_straight = racing_line.at(50).target_speed_mps;
  std::cout << "[TEST] Trajectory straight speed (s ~ 0.5m): " << v_straight << " m/s\n";
  assert(v_straight > 1.2f);

  // Tra cứu tại s ~ 1.45m: Phanh hãm đón cua (x ~ 1450, idx ~ 145)
  const float v_brake = racing_line.at(145).target_speed_mps;
  std::cout << "[TEST] Trajectory braking speed before corner (s ~ 1.45m): " << v_brake << " m/s\n";
  assert(v_brake < v_straight);

  // Tra cứu tại góc cua (idx ~ 165): Tốc độ bị giới hạn bởi độ bám góc
  const float v_corner = racing_line.at(165).target_speed_mps;
  std::cout << "[TEST] Trajectory corner apex speed: " << v_corner << " m/s\n";
  assert(v_corner < v_brake);
  assert(v_corner <=
         std::sqrt(profiler_cfg.corner_grip_mu * profiler_cfg.gravity_mps2 * 0.22f) + 0.02f);
}

void verify_trajectory_follower_tracking(const std::vector<lf::TrajectoryPoint>& racing_line,
                                         const lf::ProfilerConfig& profiler_cfg, float v_straight) {
  // Test TrajectoryFollower tracking
  const lf::PIDConfig pid_cfg{};
  const lf::ObstacleConfig obs_cfg{};
  lf::TrajectoryFollower follower(pid_cfg, profiler_cfg, obs_cfg);
  follower.set_trajectory(racing_line);
  assert(follower.has_trajectory());

  lf::SensorData in{};
  in.dt = 0.002f;
  lf::MotorCmd cmd{};
  float target_v = 0.0f;
  float target_steer = 0.0f;
  int waypoint_idx = 0;
  uint8_t bypass_state = 0;

  follower.update(lf::Pose2D{0.5f, 0.0f, 0.0f}, 1.5f, 0.003f, 0.0f, true, 0.002f, in, cmd, target_v,
                  target_steer, waypoint_idx, bypass_state);
  assert(std::abs(waypoint_idx - 50) <= 1);
  assert(std::abs(target_v - v_straight) < 0.05f);
  assert(cmd.left_pwm > 0.0f);
  assert(cmd.right_pwm > 0.0f);
}

void test_trajectory_speed_profiling_and_follower() {
  const lf::EnvironmentConfig env{};
  const lf::ProfilerConfig profiler_cfg{};
  const lf::TrajectoryPlanner planner(env, profiler_cfg);

  const auto raw_path = generate_test_curve_path();
  const auto racing_line = planner.generate_racing_line(raw_path, {});

  verify_trajectory_speed_profiling(racing_line, profiler_cfg);
  const float v_straight = racing_line.at(50).target_speed_mps;
  verify_trajectory_follower_tracking(racing_line, profiler_cfg, v_straight);
}

void test_differential_kinematics() {
  constexpr float kWheelBase = 0.115f;
  const lf::DifferentialDriveKinematics kin(kWheelBase);

  // 1. Invariant Forward Kinematics
  const float vl_orig = 0.85f;
  const float vr_orig = 0.45f;
  const auto twist = kin.forward(vl_orig, vr_orig);
  assert(std::abs(twist.linear_v_mps - 0.5f * (vl_orig + vr_orig)) < 0.0001f);
  assert(std::abs(twist.angular_w_radps - (vl_orig - vr_orig) / kWheelBase) < 0.0001f);

  // 2. Turning Direction Invariant (Poka-Yoke)
  // Left wheel faster than right wheel -> turns RIGHT (w > 0 in screen coords)
  assert(kin.forward(1.0f, 0.5f).angular_w_radps > 0.0f);
  // Right wheel faster than left wheel -> turns LEFT (w < 0 in screen coords)
  assert(kin.forward(0.5f, 1.0f).angular_w_radps < 0.0f);

  // 3. PWM Normalization Clamping (Poka-Yoke)
  assert(lf::DifferentialDriveKinematics::velocity_to_pwm(3.0f, 2.0f) == 1.0f);
  assert(lf::DifferentialDriveKinematics::velocity_to_pwm(-3.0f, 2.0f) == -1.0f);
  assert(lf::DifferentialDriveKinematics::velocity_to_pwm(1.0f, 2.0f) == 0.5f);
  assert(lf::DifferentialDriveKinematics::velocity_to_pwm(1.0f, 0.0f) == 0.0f);  // No div by zero
}

void test_closed_loop_line_convergence() {
  const lf::RobotConfig config{};
  lf::RobotCoordinator robot(config);
  robot.init();
  robot.set_mode(lf::RobotMode::kRun1Map);

  // Giả lập vòng kín 120 chu kỳ (240ms) với độ lệch ban đầu -20mm (vạch ở bên trái)
  float simulated_centroid_mm = -20.0f;
  const float dt = 0.002f;
  const lf::DifferentialDriveKinematics kin(config.kinematics.wheel_base_m);

  for (int step = 0; step < 120; ++step) {
    lf::SensorData in{};
    in.dt = dt;
    in.tof_front_m = 1.0f;

    // Chuyển centroid thành giá trị quang học 8 mắt
    for (size_t i = 0; i < lf::LineSensorConfig::kSensorCount; ++i) {
      const float dist_to_line =
        std::abs(config.line_sensor.weights_mm.at(i) - simulated_centroid_mm);
      in.ir_raw.at(i) = (dist_to_line < 10.0f) ? (1.0f - dist_to_line / 10.0f) : 0.0f;
    }

    lf::MotorCmd out{};
    robot.update(in, out);

    const float max_speed = config.profiler.max_straight_speed_mps;
    const float vl = out.left_pwm * max_speed;
    const float vr = out.right_pwm * max_speed;
    const auto twist = kin.forward(vl, vr);

    // Vận tốc góc làm thay đổi vị trí vạch tương đối so với xe
    // (w < 0 rẽ trái làm vạch từ trái dịch về tâm: delta_centroid > 0)
    simulated_centroid_mm -= twist.angular_w_radps * dt * 50.0f;
  }

  std::cout << "[TEST] Closed-loop Error Reduction: " << -20.0f << " mm -> "
            << simulated_centroid_mm << " mm\n";
  // Hệ thống kín phải hội tụ về tâm: độ lệch tuyệt đối phải giảm ít nhất 50%
  assert(std::abs(simulated_centroid_mm) < 10.0f);
}

void verify_pid_guardrails(lf::RobotCoordinator& robot) {
  // Thử gán hệ số âm hoặc quá lớn -> Poka-Yoke phải clamp về khoảng an toàn
  robot.set_line_pid_gains(-1.0f, -0.5f, -0.05f);
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  robot.get_line_pid_gains(kp, ki, kd);
  assert(kp >= 0.005f);
  assert(ki >= 0.0f);
  assert(kd >= 0.0005f);
}

void verify_speed_guardrails(lf::RobotCoordinator& robot) {
  // Thử gán tốc độ âm hoặc phi lý
  robot.set_tactical_speeds(-0.5f, 100.0f);
  assert(robot.get_config().profiler.safe_map_speed_mps >= 0.2f);
  assert(robot.get_config().profiler.max_straight_speed_mps <= 4.0f);
}

void verify_obstacle_guardrails(lf::RobotCoordinator& robot) {
  // Thử gán cự ly né vật cản âm hoặc phi lý
  robot.set_obstacle_parameters(-0.5f, -1.0f, 10.0f);
  float d_front = 0.0f;
  float d_clear = 0.0f;
  float d_len = 0.0f;
  robot.get_obstacle_parameters(d_front, d_clear, d_len);
  assert(d_front >= 0.08f);
  assert(d_front <= 0.60f);
  assert(d_clear >= 0.08f);
  assert(d_clear <= 0.35f);
  assert(d_len >= 0.30f);
  assert(d_len <= 1.50f);
}

void test_tactical_safety_guardrails() {
  const lf::RobotConfig config{};
  lf::RobotCoordinator robot(config);
  verify_pid_guardrails(robot);
  verify_speed_guardrails(robot);
  verify_obstacle_guardrails(robot);
}

void test_profiler_live_reoptimization() {
  lf::ProfilerConfig config{};
  config.max_accel_mps2 = 5.0f;
  config.max_brake_mps2 = 5.0f;
  config.max_straight_speed_mps = 2.0f;
  lf::TrajectoryPlanner const planner1(lf::EnvironmentConfig{}, config);

  std::vector<lf::TrajectoryPoint> raw_path;
  for (int x = 0; x <= 4000; x += 20) {
    raw_path.push_back({static_cast<float>(x), 0.0f, 1.0f, 0.0f});
  }

  auto traj1 = planner1.generate_racing_line(raw_path, {});
  const float v1 = traj1.at(100).target_speed_mps;

  // Cập nhật tốc độ tối đa tăng lên 3.5 m/s
  config.max_straight_speed_mps = 3.5f;
  lf::TrajectoryPlanner const planner2(lf::EnvironmentConfig{}, config);
  auto traj2 = planner2.generate_racing_line(raw_path, {});
  const float v2 = traj2.at(100).target_speed_mps;

  std::cout << "[TEST] Trajectory Live Re-opt: " << v1 << " m/s -> " << v2 << " m/s\n";
  assert(v2 > v1);
  assert(v1 <= 2.001f);
  assert(v2 > 2.2f);
}

void test_reset_pose_preserves_map() {
  const lf::RobotConfig config{};
  lf::RobotCoordinator robot(config);
  robot.init();
  robot.set_mode(lf::RobotMode::kRun1Map);

  lf::SensorData in{};
  in.dt = 0.016f;
  in.enc_left_ticks = 0;
  in.enc_right_ticks = 0;
  in.tof_front_m = 1.0f;
  in.tof_left_m = 1.0f;
  in.tof_right_m = 1.0f;
  in.ir_raw.at(3) = 1.0f;
  in.ir_raw.at(4) = 1.0f;
  lf::MotorCmd out{};

  // Run for 30 steps to accumulate trace points (> 10 points)
  for (int i = 1; i <= 30; ++i) {
    in.enc_left_ticks = i * 200;
    in.enc_right_ticks = i * 200;
    robot.update(in, out);
  }
  assert(!robot.get_recorded_trace().empty());

  // Phát hiện vạch đích sau 1.2m
  for (size_t i = 0; i < 8; ++i) {
    in.ir_raw.at(i) = 1.0f;
  }
  robot.update(in, out);

  // Gọi reset_pose(): Chỉ reset pose và odometry, giữ nguyên map đã học
  robot.reset_pose();
  assert(robot.get_mode() == lf::RobotMode::kIdle);
  assert(robot.get_telemetry().total_distance_m == 0.0f);
  assert(!robot.get_recorded_trace().empty());

  // Nếu chuyển sang RUN_2_SPRINT, bộ follower vẫn giữ nguyên quỹ đạo
  robot.set_mode(lf::RobotMode::kRun2Sprint);
  assert(robot.get_mode() == lf::RobotMode::kRun2Sprint);
}

void test_launch_velocity_fix() {
  const lf::EnvironmentConfig env{};
  const lf::ProfilerConfig config{};
  const lf::TrajectoryPlanner planner(env, config);

  std::vector<lf::TrajectoryPoint> raw_path;
  for (int x = 0; x <= 2000; x += 10) {
    raw_path.push_back({static_cast<float>(x), 0.0f, 1.0f, 0.0f});
  }

  const auto racing_line = planner.generate_racing_line(raw_path, {});
  assert(!racing_line.empty());

  const float v_launch = racing_line.front().target_speed_mps;
  std::cout << "[TEST] Launch Velocity tai s = 0.0m: " << v_launch
            << " m/s (Min required: " << config.launch_min_speed_mps << " m/s)\n";
  assert(v_launch >= config.launch_min_speed_mps);
}

void test_trajectory_planner_obstacle_bypass() {
  const lf::EnvironmentConfig env{};
  const lf::ProfilerConfig profiler_cfg{};
  const lf::TrajectoryPlanner planner(env, profiler_cfg);

  std::vector<lf::TrajectoryPoint> raw_path;
  for (int x = 0; x <= 1000; x += 10) {
    raw_path.push_back({static_cast<float>(x), 500.0f, 1.0f, 0.0f});
  }

  // Dat 1 vat can tai (500, 500) xoay goc 45 do chan ngang vach
  const std::vector<lf::ObstacleGeometry> obstacles = {{500.0f, 500.0f, 105.0f, 60.0f, 45.0f}};

  const auto racing_line = planner.generate_racing_line(raw_path, obstacles);
  assert(racing_line.size() == raw_path.size());

  // Diem dau va diem cuoi phai tiep tuyen voi tim duong goc (C2 boundary)
  assert(std::abs(racing_line.front().y_mm - 500.0f) < 0.1f);
  assert(std::abs(racing_line.back().y_mm - 500.0f) < 0.1f);

  // Diem tai tam vat can (x = 500) phai duoc ne thoat khoi R_clearance ~ 155mm
  const float dist_at_center = std::abs(racing_line.at(50).y_mm - 500.0f);
  std::cout << "[TEST] Chicane Offset tai tam vat can xoay 45°: " << dist_at_center
            << " mm (Required clearance: " << env.clearance_radius_mm() << " mm)\n";
  assert(dist_at_center >= env.clearance_radius_mm());

  // Toc do tai diem bat dau phai thoa man launch_min_speed_mps
  assert(racing_line.front().target_speed_mps >= profiler_cfg.launch_min_speed_mps);
}

void test_rich_telemetry_and_wheel_velocities() {
  const lf::RobotConfig config{};
  lf::RobotCoordinator robot(config);
  robot.init();
  robot.set_mode(lf::RobotMode::kRun1Map);

  lf::SensorData in{};
  in.dt = 0.020f;
  in.enc_left_ticks = 56;
  in.enc_right_ticks = 112;
  in.tof_front_m = 0.45f;
  in.tof_left_m = 0.60f;
  in.tof_right_m = 0.55f;

  lf::MotorCmd out{};
  robot.update(in, out);

  // Chu ky thu 2 de tinh delta encoder ticks
  in.enc_left_ticks = 112;
  in.enc_right_ticks = 224;
  robot.update(in, out);

  const auto& tel = robot.get_telemetry();
  std::cout << "[TEST] Wheel Telemetry -> L: " << tel.wheel_l.actual_speed_mps << " m/s ("
            << tel.wheel_l.enc_ticks << " ticks), R: " << tel.wheel_r.actual_speed_mps << " m/s ("
            << tel.wheel_r.enc_ticks << " ticks)\n";
  assert(tel.wheel_l.enc_ticks == 112);
  assert(tel.wheel_r.enc_ticks == 224);
  assert(tel.wheel_l.actual_speed_mps > 0.0f);
  assert(tel.wheel_r.actual_speed_mps > tel.wheel_l.actual_speed_mps);
  assert(std::abs(tel.obstacle.tof_front_m - 0.45f) < 0.001f);
}

void simulate_run1_mapping_with_obstacle(lf::RobotCoordinator& robot, lf::SensorData& in,
                                         lf::MotorCmd& out) {
  for (int step = 0; step < 120; ++step) {
    in.enc_left_ticks += 50;
    in.enc_right_ticks += 50;
    // Step 50: Detect obstacle at 0.25m ahead
    in.tof_front_m = (step == 50) ? 0.25f : 1.0f;
    robot.update(in, out);
  }
}

void simulate_run1_finish_crossing(lf::RobotCoordinator& robot, lf::SensorData& in,
                                   lf::MotorCmd& out) {
  for (size_t i = 0; i < lf::LineSensorConfig::kSensorCount; ++i) {
    in.ir_raw.at(i) = 1.0f;
  }
  for (int step = 0; step < 50; ++step) {
    in.enc_left_ticks += 50;
    in.enc_right_ticks += 50;
    robot.update(in, out);
  }
}

void simulate_run2_tracking(lf::RobotCoordinator& robot, lf::SensorData& in, lf::MotorCmd& out) {
  in.ir_raw.at(3) = 1.0f;
  in.ir_raw.at(4) = 1.0f;
  for (int step = 0; step < 50; ++step) {
    in.enc_left_ticks += 25;
    in.enc_right_ticks += 25;
    robot.update(in, out);
    // Anti-spin: Wheels maintain forward traction
    assert(out.left_pwm > 0.0f);
    assert(out.right_pwm > 0.0f);
  }
}

void test_coordinator_global_pipeline_with_obstacle() {
  const lf::RobotConfig config{};
  lf::RobotCoordinator robot(config);
  robot.init();
  robot.set_mode(lf::RobotMode::kRun1Map);

  lf::SensorData in{};
  in.dt = 0.002f;
  in.tof_front_m = 1.0f;
  in.tof_left_m = 1.0f;
  in.tof_right_m = 1.0f;
  // Centerline tracking (sensors 3 & 4 on)
  in.ir_raw.at(3) = 1.0f;
  in.ir_raw.at(4) = 1.0f;

  lf::MotorCmd out{};

  // 1. Run 1: Robot drives along line
  simulate_run1_mapping_with_obstacle(robot, in, out);

  // Verify obstacle was detected and clustered into map
  assert(!robot.get_detected_obstacles().empty());
  assert(!robot.get_recorded_trace().empty());

  // 2. Finish line crossing in Run 1 (all sensors lit, dist > 1.0m)
  simulate_run1_finish_crossing(robot, in, out);

  // Finish trigger: Coordinator auto-generates global racing line with QP & profiler
  assert(!robot.get_racing_trajectory().empty());
  assert(robot.get_mode() == lf::RobotMode::kIdle);

  // 3. Switch to Run 2 Sprint
  robot.reset_pose();
  robot.set_mode(lf::RobotMode::kRun2Sprint);
  assert(robot.get_mode() == lf::RobotMode::kRun2Sprint);

  // Run 2: Track trajectory with Stanley controller
  simulate_run2_tracking(robot, in, out);

  std::cout
    << "[TEST SEAM 4] Coordinator Run 1 Map -> Auto QP Gen -> Run 2 Stanley Sprint: Verified\n";
}

void test_telemetry_packet_serialization_and_checksum() {
  lf::TelemetryPacket pkt{};
  pkt.timestamp_ms = 12345;
  pkt.enc_left_ticks = 1000;
  pkt.enc_right_ticks = 1050;
  pkt.centroid_pos_mm = 5.2f;
  pkt.battery_voltage_v = 8.24f;
  pkt.finalize();

  assert(pkt.is_valid());
  assert(pkt.magic == lf::kTelemetryMagic);
  assert(sizeof(lf::TelemetryPacket) == 84);

  // Tham nhieu thu mot byte bat ky -> Checksum phai phat hien va bao khong hop le
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  auto* raw = reinterpret_cast<uint8_t*>(&pkt);
  raw[10] ^= 0xFF;
  assert(!pkt.is_valid());

  // Khoi phuc byte -> Hop le tro lai
  raw[10] ^= 0xFF;
  assert(pkt.is_valid());
  std::cout << "[TEST] Telemetry Packet -> 84 bytes packed, Checksum validated: 0x" << std::hex
            << pkt.checksum << std::dec << "\n";
}

void test_run1_corner_deceleration_and_line_loss_latch() {
  const lf::RobotConfig config{};
  lf::RobotCoordinator robot(config);
  robot.init();
  robot.set_mode(lf::RobotMode::kRun1Map);

  // 1. Straight line: line at center (0 mm)
  lf::SensorData in{};
  in.dt = 0.02f;
  in.tof_front_m = 1.0f;
  in.ir_raw.at(3) = 1.0f;
  in.ir_raw.at(4) = 1.0f;

  lf::MotorCmd out{};
  robot.update(in, out);
  assert(std::abs(out.left_pwm - out.right_pwm) < 0.001f);
  const float straight_pwm = out.left_pwm;

  // 2. Sharp turn: error is high (line under sensor 0, outermost left)
  in.ir_raw = {};
  in.ir_raw.at(0) = 1.0f;
  robot.update(in, out);

  const float corner_avg_pwm = (out.left_pwm + out.right_pwm) * 0.5f;
  assert(corner_avg_pwm <= straight_pwm);
  assert(out.right_pwm > out.left_pwm);  // Steer left: right wheel faster

  // 3. Safe Stop on Line Loss: line leaves field of view completely
  in.ir_raw = {};
  robot.update(in, out);

  assert(robot.get_telemetry().line_lost);
  // Must stop motors safely to prevent spin-outs and wild runaway
  assert(out.left_pwm == 0.0f);
  assert(out.right_pwm == 0.0f);

  std::cout << "[TEST] Run 1 Line Following & Safe Stop -> Straight PWM: " << straight_pwm
            << " | Corner Avg PWM: " << corner_avg_pwm << " | Stopped on line loss\n";
}

void simulate_run1_enter_finish_zone(lf::RobotCoordinator& robot, lf::SensorData& in,
                                     lf::MotorCmd& out) {
  // 1. Mắt đọc thấy ô đen đích: robot tiếp tục chạy kéo thân xe vào trong ô và tối ưu racing line
  for (size_t i = 0; i < 8; ++i) {
    in.ir_raw.at(i) = 1.0f;
  }
  robot.update(in, out);

  assert(robot.get_mode() == lf::RobotMode::kRun1Map);
  assert(!out.active_brake);
  assert(out.left_pwm > 0.0f);
  assert(robot.get_follower().has_trajectory());
}

void simulate_run1_stop_in_finish_zone(lf::RobotCoordinator& robot, lf::SensorData& in,
                                       lf::MotorCmd& out) {
  // Di chuyển tiếp trong finish zone
  in.enc_left_ticks += 262;
  in.enc_right_ticks += 262;
  robot.update(in, out);
  assert(robot.get_mode() == lf::RobotMode::kRun1Map);
  assert(!out.active_brake);

  // Vượt qua chiều dài ô đích -> Dừng hẳn
  in.enc_left_ticks += 600;
  in.enc_right_ticks += 600;
  robot.update(in, out);
  assert(robot.get_mode() == lf::RobotMode::kIdle);
  assert(out.active_brake);
  assert(out.left_pwm == 0.0f);
}

void simulate_run2_sprint_start(lf::RobotCoordinator& robot, lf::SensorData& in,
                                lf::MotorCmd& out) {
  // Chuyển sang Run 2 Sprint: chạy với trajectory đã tối ưu
  robot.set_mode(lf::RobotMode::kRun2Sprint);
  in.enc_left_ticks = 0;
  in.enc_right_ticks = 0;
  in.ir_raw = {};
  in.ir_raw.at(3) = 1.0f;
  in.ir_raw.at(4) = 1.0f;
  robot.update(in, out);

  in.enc_left_ticks = 1000;
  in.enc_right_ticks = 1000;
  robot.update(in, out);
  assert(out.left_pwm > 0.05f);
  assert(!out.active_brake);
}

void test_finish_zone_pull_in_and_sprint2_completion() {
  const lf::RobotConfig config{};
  lf::RobotCoordinator robot(config);
  robot.init();
  robot.set_mode(lf::RobotMode::kRun1Map);

  lf::SensorData in{};
  in.dt = 0.02f;
  in.enc_left_ticks = 0;
  in.enc_right_ticks = 0;
  in.tof_front_m = 1.0f;
  in.ir_raw.at(3) = 1.0f;
  in.ir_raw.at(4) = 1.0f;
  lf::MotorCmd out{};
  robot.update(in, out);

  // Chạy từng bước vượt qua min_finish_arm_dist_m (1.0m ~ 5243 ticks) và ghi nhận >10 waypoint
  for (int step = 1; step <= 25; ++step) {
    in.enc_left_ticks = step * 250;
    in.enc_right_ticks = step * 250;
    robot.update(in, out);
  }
  assert(robot.get_recorded_trace().size() >= 10);

  simulate_run1_enter_finish_zone(robot, in, out);
  simulate_run1_stop_in_finish_zone(robot, in, out);
  simulate_run2_sprint_start(robot, in, out);

  std::cout << "[TEST] Finish Zone Pull-in & Sprint 2 Completion -> Verified\n";
}

void test_speed_scaled_pid_attenuation() {
  lf::PIDConfig config{};
  config.kp = 0.05f;
  config.ki = 0.0f;
  config.kd = 0.0f;
  config.out_min = -10.0f;
  config.out_max = 10.0f;
  config.speed_scale_min = 0.35f;
  config.speed_scale_max = 1.80f;
  config.speed_scale_v_ref = 0.80f;

  lf::PIDController pid(config);

  const float error = 10.0f;
  const float dt = 0.002f;

  // At reference speed (0.8 m/s): scale = 1.0 -> output = 0.05 * 1.0 * 10 = 0.5
  const float u_ref = pid.update_speed_scaled(error, dt, 0.80f);
  pid.reset();

  // At low speed (0.4 m/s): scale = 0.8 / 0.4 = 2.0 -> clamped to 1.80 -> output = 0.05 * 1.8 * 10
  // = 0.9
  const float u_low = pid.update_speed_scaled(error, dt, 0.40f);
  pid.reset();

  // At high speed (2.4 m/s): scale = 0.8 / 2.4 = 0.333 -> clamped to 0.35 -> output = 0.05 * 0.35 *
  // 10 = 0.175
  const float u_high = pid.update_speed_scaled(error, dt, 2.40f);

  assert(u_low > u_ref);
  assert(u_high < u_ref);
  assert(std::abs(u_low / u_ref - 1.80f) < 0.05f);
  assert(std::abs(u_high / u_ref - 0.35f) < 0.05f);

  std::cout << "[TEST] Speed-Scaled PID: u_low=" << u_low << ", u_ref=" << u_ref
            << ", u_high=" << u_high << " -> Attenuation verified\n";
}

void test_kinematic_curvature_mapping() {
  const lf::RobotConfig config{};
  lf::RobotCoordinator robot(config);
  robot.init();
  robot.set_mode(lf::RobotMode::kRun1Map);

  lf::SensorData in{};
  in.dt = 0.02f;
  in.tof_front_m = 1.0f;
  // Centroid centered (error = 0)
  in.ir_raw.at(3) = 1.0f;
  in.ir_raw.at(4) = 1.0f;

  lf::MotorCmd out{};
  // Simulate turning curve where Left wheel turns slower than Right wheel (turning left)
  for (int step = 0; step < 25; ++step) {
    in.enc_left_ticks += 20;   // slower
    in.enc_right_ticks += 50;  // faster
    robot.update(in, out);
  }

  // The coordinator should recognize the curve from wheel odometry
  assert(robot.get_telemetry().total_distance_m > 0.05f);
  assert(robot.get_telemetry().angular_velocity_radps != 0.0f);
  assert(!robot.get_recorded_trace().empty());
  std::cout << "[TEST] Kinematic Curvature Trace Points: count="
            << robot.get_recorded_trace().size() << " -> Verified\n";
}

void test_reactive_sprint_braking_cap() {
  lf::RobotConfig config{};
  config.profiler.max_straight_speed_mps = 2.50f;
  config.obstacle.front_detect_dist_m = 0.28f;
  config.obstacle.approach_stop_dist_m = 0.07f;
  config.profiler.max_brake_mps2 = 4.50f;

  lf::RobotCoordinator robot(config);
  robot.init();
  // Set sprint mode without mapping run (unoptimized profiler -> reactive sprint)
  robot.set_mode(lf::RobotMode::kRun2Sprint);

  lf::SensorData in{};
  in.dt = 0.002f;
  in.tof_front_m = 1.0f;
  // Centroid = 0 (perfect straight line)
  in.ir_raw.at(3) = 1.0f;
  in.ir_raw.at(4) = 1.0f;

  lf::MotorCmd out{};
  robot.update(in, out);

  const float v_safe_expected = std::sqrt(2.0f * 4.50f * (0.28f - 0.07f));  // ≈ 1.375 m/s
  const float v_actual = robot.get_telemetry().current_velocity_mps;

  assert(v_actual <= v_safe_expected + 0.01f);
  assert(v_actual < config.profiler.max_straight_speed_mps);

  std::cout << "[TEST] Reactive Sprint Braking Cap: v_actual=" << v_actual
            << " <= v_safe=" << v_safe_expected << " -> Verified\n";
}

void test_sprint_corridor_and_late_apex() {
  lf::ProfilerConfig config{};
  config.safe_map_speed_mps = 0.80f;
  config.max_straight_speed_mps = 2.00f;

  lf::TrajectoryPlanner const planner(lf::EnvironmentConfig{}, config);

  std::vector<lf::TrajectoryPoint> raw_path;
  for (int x = 0; x <= 2000; x += 10) {
    const float y =
      (x > 800 && x < 1400) ? 200.0f * std::sin(static_cast<float>(x - 800) * kPi / 600.0f) : 0.0f;
    raw_path.push_back({static_cast<float>(x), y, 1.0f, 0.0f});
  }

  const auto racing_line = planner.generate_racing_line(raw_path, {});
  assert(racing_line.size() == raw_path.size());

  // Rule: Zero Off-Track Corner Cutting.
  // Every optimized point must remain within the tape boundary corridor (+/- 15mm)
  float max_lateral_dev = 0.0f;
  for (size_t i = 0; i < raw_path.size(); ++i) {
    const float dx = racing_line.at(i).x_mm - raw_path.at(i).x_mm;
    const float dy = racing_line.at(i).y_mm - raw_path.at(i).y_mm;
    const float dev = std::sqrt(dx * dx + dy * dy);
    max_lateral_dev = std::max(dev, max_lateral_dev);
  }

  std::cout << "[TEST] Corridor Relaxation Max Deviation: " << max_lateral_dev
            << " mm (Strict limit: 15.0 mm)\n";
  assert(max_lateral_dev <= 15.001f);
}

void test_sensor_area_integration_model() {
  // 3x3 Area integration modeling physical 5mm sensor aperture spot
  constexpr std::array<std::array<float, 3>, 3> kKernelWeights = {
    {{0.06f, 0.14f, 0.06f}, {0.14f, 0.20f, 0.14f}, {0.06f, 0.14f, 0.06f}}};

  // 1. Verify kernel mathematical normalization: Sum of weights == 1.0f
  float sum_weights = 0.0f;
  for (const auto& row : kKernelWeights) {
    for (const float weight : row) {
      sum_weights += weight;
    }
  }
  std::cout << "[TEST] 3x3 IR Aperture Kernel Weight Sum: " << sum_weights << "\n";
  assert(std::abs(sum_weights - 1.0f) < 1e-4f);

  // 2. Center weight must have highest contribution (Gaussian bell shape)
  assert(kKernelWeights.at(1).at(1) > kKernelWeights.at(0).at(1));
  assert(kKernelWeights.at(0).at(1) > kKernelWeights.at(0).at(0));
}

void test_global_trajectory_optimizer_qp() {
  const lf::EnvironmentConfig env{};
  lf::OptimizationConfig opt_cfg{};
  opt_cfg.max_lateral_deviation_mm = 15.0f;
  const lf::GlobalTrajectoryOptimizer optimizer(opt_cfg);

  // 1. Curve smoothing / Apex cutting test:
  // Build a track with a 90 deg corner (R = 200mm)
  std::vector<lf::TrajectoryPoint> raw_corner_track;
  for (int x = 0; x <= 1000; x += 10) {
    raw_corner_track.push_back({static_cast<float>(x), 0.0f, 1.0f, 0.0f});
  }
  constexpr int kSteps = 30;
  for (int i = 1; i <= kSteps; ++i) {
    const float th = -0.5f * kPi + (0.5f * kPi) * (static_cast<float>(i) / kSteps);
    const float cx = 1000.0f + 200.0f * std::cos(th);
    const float cy = 200.0f + 200.0f * std::sin(th);
    raw_corner_track.push_back({cx, cy, 1.0f, 0.0f});
  }
  for (int y = 210; y <= 1000; y += 10) {
    raw_corner_track.push_back({1200.0f, static_cast<float>(y), 1.0f, 0.0f});
  }

  for (size_t i = 1; i + 1 < raw_corner_track.size(); ++i) {
    const float dx1 = raw_corner_track.at(i).x_mm - raw_corner_track.at(i - 1).x_mm;
    const float dy1 = raw_corner_track.at(i).y_mm - raw_corner_track.at(i - 1).y_mm;
    const float dx2 = raw_corner_track.at(i + 1).x_mm - raw_corner_track.at(i).x_mm;
    const float dy2 = raw_corner_track.at(i + 1).y_mm - raw_corner_track.at(i).y_mm;
    const float d1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
    const float d2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
    const float dx3 = raw_corner_track.at(i + 1).x_mm - raw_corner_track.at(i - 1).x_mm;
    const float dy3 = raw_corner_track.at(i + 1).y_mm - raw_corner_track.at(i - 1).y_mm;
    const float d3 = std::sqrt(dx3 * dx3 + dy3 * dy3);
    const float cross = dx1 * dy2 - dy1 * dx2;
    const float denom = d1 * d2 * d3;
    if (denom > 1e-6f) {
      raw_corner_track.at(i).curvature_rad_m = (2.0f * std::abs(cross) / denom) * 1000.0f;
    }
  }

  const auto opt_corner = optimizer.optimize(raw_corner_track, {}, env);
  assert(opt_corner.size() == raw_corner_track.size());

  // Evaluate total squared curvature of centerline vs optimized line
  float sum_sq_curv_cl = 0.0f;
  float sum_sq_curv_opt = 0.0f;
  float max_dev_mm = 0.0f;

  for (size_t i = 1; i + 1 < raw_corner_track.size(); ++i) {
    sum_sq_curv_cl +=
      raw_corner_track.at(i).curvature_rad_m * raw_corner_track.at(i).curvature_rad_m;
    sum_sq_curv_opt += opt_corner.at(i).curvature_rad_m * opt_corner.at(i).curvature_rad_m;

    const float dx = opt_corner.at(i).x_mm - raw_corner_track.at(i).x_mm;
    const float dy = opt_corner.at(i).y_mm - raw_corner_track.at(i).y_mm;
    const float dev = std::sqrt(dx * dx + dy * dy);
    max_dev_mm = std::max(dev, max_dev_mm);
  }

  std::cout << "[TEST SEAM 1] Curvature sum sq: CL=" << sum_sq_curv_cl
            << ", OPT=" << sum_sq_curv_opt << ", Max dev=" << max_dev_mm << " mm\n";
  // The global QP must reduce total squared curvature (apex-cutting)
  assert(sum_sq_curv_opt < sum_sq_curv_cl);
  // And must stay within the legal track corridor
  assert(max_dev_mm <= opt_cfg.max_lateral_deviation_mm + 0.05f);

  // 2. Obstacle bypass test on straight track:
  std::vector<lf::TrajectoryPoint> straight_track;
  for (int x = 0; x <= 2000; x += 10) {
    straight_track.push_back({static_cast<float>(x), 0.0f, 1.0f, 0.0f});
  }
  lf::ObstacleGeometry obs{};
  obs.x_mm = 1000.0f;
  obs.y_mm = 0.0f;
  obs.width_mm = 105.0f;
  obs.height_mm = 60.0f;
  obs.angle_deg = 0.0f;

  const auto opt_obs = optimizer.optimize(straight_track, {obs}, env);
  assert(opt_obs.size() == straight_track.size());

  // Verify clearance at closest approach:
  float min_dist_to_obs = 1e9f;
  for (const auto& pt : opt_obs) {
    const float dx = pt.x_mm - obs.x_mm;
    const float dy = pt.y_mm - obs.y_mm;
    const float dist = std::sqrt(dx * dx + dy * dy);
    min_dist_to_obs = std::min(dist, min_dist_to_obs);
  }
  const float req_clearance = env.clearance_radius_mm();
  std::cout << "[TEST SEAM 1] Obstacle clearance: actual=" << min_dist_to_obs
            << " mm, required=" << req_clearance << " mm\n";
  assert(min_dist_to_obs >= req_clearance - 1.0f);
}

void test_time_optimal_velocity_profiler() {
  const lf::EnvironmentConfig env{};
  lf::ProfilerConfig profiler_cfg{};
  profiler_cfg.corner_grip_mu = 0.80f;
  profiler_cfg.gravity_mps2 = 9.81f;
  profiler_cfg.max_straight_speed_mps = 2.50f;
  profiler_cfg.max_accel_mps2 = 3.00f;
  profiler_cfg.max_brake_mps2 = 4.50f;
  profiler_cfg.safe_map_speed_mps = 0.60f;
  profiler_cfg.launch_min_speed_mps = 0.60f;

  lf::TrajectoryPlanner const planner(env, profiler_cfg);

  // Track: 2.0m straight, tight 90 deg corner (R=150mm), 2.0m straight
  std::vector<lf::TrajectoryPoint> track;
  for (int x = 0; x <= 2000; x += 10) {
    track.push_back({static_cast<float>(x), 0.0f, 1.0f, 0.0f});
  }
  constexpr int kSteps = 30;
  for (int i = 1; i <= kSteps; ++i) {
    const float th = -0.5f * kPi + (0.5f * kPi) * (static_cast<float>(i) / kSteps);
    const float cx = 2000.0f + 150.0f * std::cos(th);
    const float cy = 150.0f + 150.0f * std::sin(th);
    track.push_back({cx, cy, 1.0f, 0.0f});
  }
  for (int y = 160; y <= 2000; y += 10) {
    track.push_back({2150.0f, static_cast<float>(y), 1.0f, 0.0f});
  }

  const auto profiled = planner.generate_racing_line(track, {});
  assert(profiled.size() == track.size());

  const float mu_g = profiler_cfg.corner_grip_mu * profiler_cfg.gravity_mps2;

  // 1. Friction circle verification: a_lat(s) = v^2 * kappa <= mu * g everywhere
  float max_a_lat = 0.0f;
  for (const auto& pt : profiled) {
    const float a_lat = pt.target_speed_mps * pt.target_speed_mps * pt.curvature_rad_m;
    max_a_lat = std::max(a_lat, max_a_lat);
    // Strict Kamm friction circle limit
    assert(a_lat <= mu_g + 0.15f);
  }

  // 2. Early braking lookahead verification:
  // Corner entry is at x = 2000mm (idx = 200).
  // Top speed reached on straight:
  const float v_top = profiled.at(100).target_speed_mps;
  const float v_corner_apex = profiled.at(215).target_speed_mps;
  const float expected_brake_dist =
    (v_top * v_top - v_corner_apex * v_corner_apex) / (2.0f * profiler_cfg.max_brake_mps2);

  // Find where braking begins (speed drops below 98% of peak straight speed)
  size_t brake_start_idx = 0;
  for (size_t i = 100; i < 200; ++i) {
    if (profiled.at(i).target_speed_mps < 0.98f * v_top) {
      brake_start_idx = i;
      break;
    }
  }
  const float actual_brake_dist = profiled.at(215).s_m - profiled.at(brake_start_idx).s_m;

  std::cout << "[TEST SEAM 2] Max a_lat: " << max_a_lat << " <= mu*g=" << mu_g
            << " m/s^2 | Brake dist: actual=" << actual_brake_dist
            << " m, kinematic=" << expected_brake_dist << " m" << '\n';

  // Total braking distance must match kinematic braking distance
  assert(actual_brake_dist >= expected_brake_dist - 0.02f);

  // 3. Time optimality: total lap time sum(ds / v) must be significantly faster than safe speed
  float lap_time_opt = 0.0f;
  float lap_time_safe = 0.0f;
  for (size_t i = 1; i < profiled.size(); ++i) {
    const float ds = profiled.at(i).s_m - profiled.at(i - 1).s_m;
    lap_time_opt += ds / std::max(profiled.at(i).target_speed_mps, 0.1f);
    lap_time_safe += ds / profiler_cfg.safe_map_speed_mps;
  }
  std::cout << "[TEST SEAM 2] Lap time: OPT=" << lap_time_opt << " s vs SAFE=" << lap_time_safe
            << " s\n";
  assert(lap_time_opt < 0.70f * lap_time_safe);  // At least 30% faster
}

void test_stanley_continuous_tracker() {
  lf::StanleyConfig cfg{};
  cfg.k_cross_track = 4.5f;
  cfg.k_soft = 0.4f;
  cfg.k_heading = 1.0f;
  cfg.wheel_base_m = 0.115f;

  const lf::StanleyTracker tracker(cfg);

  // 1. Straight path along x axis (y = 0)
  std::vector<lf::TrajectoryPoint> straight_path;
  for (int x = 0; x <= 3000; x += 10) {
    straight_path.push_back(
      {static_cast<float>(x), 0.0f, 1.50f, 0.0f, static_cast<float>(x) * 0.001f, 0.0f});
  }

  // Initial robot pose with 50mm lateral offset (y = 0.05m), heading aligned (0 rad)
  lf::Pose2D pose{0.0f, 0.050f, 0.0f};
  constexpr float kDt = 0.002f;
  constexpr int kSteps = 500;  // 1.0 second simulation at 500 Hz

  float const initial_error = std::abs(pose.y_m);
  bool no_wheel_reversal = true;

  for (int i = 0; i < kSteps; ++i) {
    const auto cmd = tracker.update(pose, 1.50f, straight_path);

    if (cmd.left_wheel_mps < 0.0f || cmd.right_wheel_mps < 0.0f) {
      no_wheel_reversal = false;
    }

    // Kinematic forward simulation (SSoT)
    const lf::DifferentialDriveKinematics kin(cfg.wheel_base_m);
    const auto twist = kin.forward(cmd.left_wheel_mps, cmd.right_wheel_mps);
    const float v = twist.linear_v_mps;
    const float w = twist.angular_w_radps;

    pose.x_m += v * std::cos(pose.heading_rad) * kDt;
    pose.y_m += v * std::sin(pose.heading_rad) * kDt;
    pose.heading_rad += w * kDt;
  }
  const float final_error = std::abs(pose.y_m);

  std::cout << "[TEST SEAM 3] Closed-loop convergence: " << initial_error * 1000.0f << " mm -> "
            << final_error * 1000.0f << " mm in 1.0s" << '\n';
  // Cross-track error must decay to < 3mm within 1.0s
  assert(final_error < 0.003f);
  // Both wheels must maintain forward traction without negative wheel reversal
  assert(no_wheel_reversal);
}

std::vector<lf::TrajectoryPoint> generate_test_oval_track() {
  std::vector<lf::TrajectoryPoint> track;
  // Bottom straight: (500, 1300) -> (1900, 1300)
  for (int x = 500; x <= 1900; x += 10) {
    track.push_back({static_cast<float>(x), 1300.0f, 1.0f, 0.0f});
  }
  // Right semicircle: R = 350mm to (1900, 600)
  for (int i = 1; i <= 50; ++i) {
    const float phi = kPi * (static_cast<float>(i) / 50.0f);
    track.push_back(
      {1900.0f + 350.0f * std::sin(phi), 950.0f + 350.0f * std::cos(phi), 1.0f, 0.0f});
  }
  // Top straight: (1900, 600) -> (500, 600)
  for (int x = 1890; x >= 500; x -= 10) {
    track.push_back({static_cast<float>(x), 600.0f, 1.0f, 0.0f});
  }
  // Left semicircle: R = 350mm back to (500, 1300)
  for (int i = 1; i <= 50; ++i) {
    const float phi = kPi + kPi * (static_cast<float>(i) / 50.0f);
    track.push_back({500.0f + 350.0f * std::sin(phi), 950.0f + 350.0f * std::cos(phi), 1.0f, 0.0f});
  }
  return track;
}

float compute_max_obstacle_curvature(const std::vector<lf::TrajectoryPoint>& opt) {
  float max_obs_kappa = 0.0f;
  for (size_t i = 1; i + 1 < opt.size(); ++i) {
    if (opt.at(i).x_mm < 800.0f || opt.at(i).x_mm > 1400.0f || opt.at(i).y_mm <= 1000.0f) {
      continue;
    }
    const float dx1 = opt.at(i).x_mm - opt.at(i - 1).x_mm;
    const float dy1 = opt.at(i).y_mm - opt.at(i - 1).y_mm;
    const float dx2 = opt.at(i + 1).x_mm - opt.at(i).x_mm;
    const float dy2 = opt.at(i + 1).y_mm - opt.at(i).y_mm;
    const float cross = dx1 * dy2 - dy1 * dx2;
    const float d1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
    const float d2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
    const float dx3 = opt.at(i + 1).x_mm - opt.at(i - 1).x_mm;
    const float dy3 = opt.at(i + 1).y_mm - opt.at(i - 1).y_mm;
    const float d3 = std::sqrt(dx3 * dx3 + dy3 * dy3);
    if (d1 * d2 * d3 > 1e-6f) {
      const float k = (2.0f * std::abs(cross) / (d1 * d2 * d3)) * 1000.0f;
      max_obs_kappa = std::max(max_obs_kappa, k);
    }
  }
  return max_obs_kappa;
}

void test_closed_loop_obstacle_hyperplane_isolation() {
  const lf::EnvironmentConfig env{};
  const lf::GlobalTrajectoryOptimizer optimizer;

  const auto track = generate_test_oval_track();

  // Obstacle on bottom straight at (1100, 1300)
  lf::ObstacleGeometry obs{};
  obs.x_mm = 1100.0f;
  obs.y_mm = 1300.0f;
  obs.width_mm = 105.0f;
  obs.height_mm = 60.0f;

  const auto opt_no_obs = optimizer.optimize(track, {}, env);
  const auto opt = optimizer.optimize(track, {obs}, env);
  assert(opt.size() == track.size());
  assert(opt_no_obs.size() == track.size());

  // 1. Verify Top Straight (y = 600, 700mm away) has ZERO obstacle distortion vs unhindered run
  float max_top_diff = 0.0f;
  for (size_t i = 0; i < opt.size(); ++i) {
    if (std::abs(track.at(i).y_mm - 600.0f) < 1.0f) {
      const float diff = std::abs(opt.at(i).y_mm - opt_no_obs.at(i).y_mm);
      max_top_diff = std::max(max_top_diff, diff);
    }
  }
  std::cout << "[TEST] Hyperplane isolation: Top straight obstacle distortion = " << max_top_diff
            << " mm (Strict limit: < 0.1 mm)\n";
  assert(max_top_diff < 0.1f);

  // 2. Verify obstacle clearance around (1100, 1300)
  float min_dist_to_obs = 1e9f;
  for (const auto& pt : opt) {
    const float dx = pt.x_mm - obs.x_mm;
    const float dy = pt.y_mm - obs.y_mm;
    const float d = std::sqrt(dx * dx + dy * dy);
    min_dist_to_obs = std::min(d, min_dist_to_obs);
  }
  const float req_clear = env.clearance_radius_mm();
  std::cout << "[TEST] Closed-loop obstacle clearance = " << min_dist_to_obs
            << " mm (Required: " << req_clear << " mm)\n";
  assert(min_dist_to_obs >= req_clear - 2.0f);

  // 3. Verify maximum curvature around obstacle: kappa < 5.0 rad/m (kinodynamically smooth)
  const float max_obs_kappa = compute_max_obstacle_curvature(opt);
  std::cout << "[TEST] Peak kinodynamic curvature around obstacle = " << max_obs_kappa
            << " rad/m (Safe limit: < 5.0 rad/m)\n";
  assert(max_obs_kappa < 5.0f);
}

void test_run1_reactive_obstacle_evasion() {
  const lf::PIDConfig pid_cfg{0.045f, 0.0005f, 0.008f, -1.0f, 1.0f, 0.7f};
  const lf::ProfilerConfig prof_cfg{};
  const lf::ObstacleConfig obs_cfg{};

  lf::TrajectoryFollower follower(pid_cfg, prof_cfg, obs_cfg);

  lf::SensorData in{};
  lf::MotorCmd out{};
  float target_v = 0.0f;
  float target_steer = 0.0f;
  int waypoint_idx = 0;
  uint8_t bypass_state = 0;

  // 1. Normal line tracking (no obstacle)
  follower.update(lf::Pose2D{0.0f, 0.0f, 0.0f}, 0.8f, 0.01f, 0.0f, true, 0.002f, in, out, target_v,
                  target_steer, waypoint_idx, bypass_state);
  assert(bypass_state == 0);

  // 2. Obstacle detected by front ToF at 250mm
  in.tof_front_m = 0.25f;
  lf::Pose2D pose{0.0f, 0.0f, 0.0f};
  float max_heading_dev_rad = 0.0f;
  float max_lateral_clearance_m = 0.0f;
  constexpr float kDt = 0.010f;
  constexpr float kWheelBase = 0.115f;

  for (int step = 0; step < 220; ++step) {
    if (step > 5) {
      in.tof_front_m = 1.0f;  // Obstacle cleared from front sensor cone
    }
    follower.update(pose, 0.45f, 0.45f * kDt, 0.0f, false, kDt, in, out, target_v, target_steer,
                    waypoint_idx, bypass_state);

    // Forward differential drive kinematics integration
    const float max_speed = prof_cfg.max_straight_speed_mps;
    const float vl = out.left_pwm * max_speed;
    const float vr = out.right_pwm * max_speed;
    const float v = 0.5f * (vl + vr);
    const float w = (vl - vr) / kWheelBase;

    pose.x_m += v * std::cos(pose.heading_rad) * kDt;
    pose.y_m += v * std::sin(pose.heading_rad) * kDt;
    pose.heading_rad += w * kDt;

    max_heading_dev_rad = std::max(max_heading_dev_rad, std::abs(pose.heading_rad));
    max_lateral_clearance_m = std::max(max_lateral_clearance_m, -pose.y_m);
  }

  // Verify Mathematical Invariants of Continuous C2 Evasion:
  // 1. Heading is strictly bounded (peak angle ~42.8 deg, never spins 180 or even 50 deg)
  assert(max_heading_dev_rad < 0.85f);  // < 48.7 deg
  // 2. Lateral clearance peaks at >= 140mm (guarantees safe passage around 105x60mm obstacle)
  assert(max_lateral_clearance_m >= 0.14f);
  // 3. Evasion completes and robot returns to centerline
  assert(bypass_state == 0);
  assert(std::abs(pose.y_m) < 0.040f);  // within 40mm of centerline

  std::cout << "[TEST] Run 1 Continuous C2 Evasion: Max heading dev = "
            << (max_heading_dev_rad * lf::constants::kRadToDeg)
            << " deg, Peak clearance = " << (max_lateral_clearance_m * 1000.0f)
            << " mm, Final Y err = " << (std::abs(pose.y_m) * 1000.0f) << " mm\n";
}

void simulate_relay_oscillation(lf::RelayTuner& tuner, float target_amp_mm, float target_period_s,
                                float sim_time_s, float dt) {
  const int total_steps = static_cast<int>(sim_time_s / dt);
  const float omega = lf::constants::kTwoPi / target_period_s;
  for (int step = 0; step < total_steps; ++step) {
    const float t = static_cast<float>(step) * dt;
    const float centroid = target_amp_mm * std::sin(omega * t);
    tuner.update(centroid, dt);
  }
}

void assert_tuner_result_valid(const lf::TunerResult& res, float target_amp_mm,
                               float target_period_s) {
  assert(res.converged);
  assert(res.cycles_completed >= 4);
  assert(std::abs(res.oscillation_amp_mm - target_amp_mm) < 0.5f);
  assert(std::abs(res.ultimate_period_s - target_period_s) < 0.02f);
  assert(res.kp >= 0.005f && res.kp <= 0.150f);
  assert(res.kd >= 0.0005f && res.kd <= 0.050f);
}

lf::TunerResult verify_relay_tuner_synthetic_oscillation() {
  lf::RelayTuner tuner(0.12f, 1.0f);
  assert(!tuner.is_converged());

  constexpr float kDt = 0.002f;
  constexpr float kSimTimeS = 3.5f;
  constexpr float kTargetAmpMm = 14.0f;
  constexpr float kTargetPeriodS = 0.50f;

  simulate_relay_oscillation(tuner, kTargetAmpMm, kTargetPeriodS, kSimTimeS, kDt);

  assert(tuner.is_converged());
  const auto res = tuner.get_result();
  assert_tuner_result_valid(res, kTargetAmpMm, kTargetPeriodS);
  return res;
}

void simulate_coordinator_tuning_loop(lf::RobotCoordinator& robot, float target_amp_mm,
                                      float target_period_s, float sim_time_s, float dt) {
  const int total_steps = static_cast<int>(sim_time_s / dt);
  const float omega = lf::constants::kTwoPi / target_period_s;
  lf::SensorData in{};
  in.dt = dt;
  in.tof_front_m = 1.0f;
  lf::MotorCmd out{};

  for (int step = 0; step < total_steps; ++step) {
    const float t = static_cast<float>(step) * dt;
    const float centroid = target_amp_mm * std::sin(omega * t);
    in.ir_raw = {};
    if (centroid >= 0.0f) {
      in.ir_raw.at(4) = 0.8f;
      in.ir_raw.at(5) = 0.5f;
    } else {
      in.ir_raw.at(3) = 0.8f;
      in.ir_raw.at(2) = 0.5f;
    }
    robot.update(in, out);
    if (robot.get_mode() == lf::RobotMode::kIdle) {
      break;
    }
  }
}

void verify_coordinator_auto_tune_transition() {
  constexpr float kDt = 0.002f;
  constexpr float kSimTimeS = 3.5f;
  constexpr float kTargetAmpMm = 14.0f;
  constexpr float kTargetPeriodS = 0.50f;

  const lf::RobotConfig config{};
  lf::RobotCoordinator robot(config);
  robot.init();
  robot.set_mode(lf::RobotMode::kAutoTune);
  assert(robot.get_mode() == lf::RobotMode::kAutoTune);

  simulate_coordinator_tuning_loop(robot, kTargetAmpMm, kTargetPeriodS, kSimTimeS, kDt);

  assert(robot.get_mode() == lf::RobotMode::kIdle);
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  robot.get_line_pid_gains(kp, ki, kd);
  assert(kp >= 0.005f);
  assert(kd >= 0.0005f);
}

void test_relay_auto_tuner() {
  const auto res = verify_relay_tuner_synthetic_oscillation();
  verify_coordinator_auto_tune_transition();

  std::cout << "[TEST] Relay Auto-Tuner: Converged with Kp=" << res.kp << ", Kd=" << res.kd
            << ", Tu=" << res.ultimate_period_s << "s, Amp=" << res.oscillation_amp_mm << "mm\n";
}

void test_sequential_obstacle_evasion() {
  const lf::PIDConfig pid_cfg{0.008f, 0.0f, 0.001f, -1.0f, 1.0f, 0.80f};
  const lf::ProfilerConfig prof_cfg{};
  const lf::ObstacleConfig obs_cfg{};

  lf::TrajectoryFollower follower(pid_cfg, prof_cfg, obs_cfg);

  lf::SensorData in{};
  lf::MotorCmd out{};
  float target_v = 0.0f;
  float target_steer = 0.0f;
  int waypoint_idx = 0;
  uint8_t bypass_state = 0;

  lf::Pose2D pose{0.0f, 0.0f, 0.0f};
  constexpr float kDt = 0.010f;
  constexpr float kWheelBase = 0.115f;
  int evasions_triggered = 0;

  // Simulate vehicle moving forward on a line with two sequential obstacles:
  // Obstacle 1 at x = 0.35m
  // Obstacle 2 at x = 1.60m
  for (int step = 0; step < 600; ++step) {
    const float dist_to_obs1 = 0.35f - pose.x_m;
    const float dist_to_obs2 = 1.60f - pose.x_m;

    if (dist_to_obs1 > 0.02f && dist_to_obs1 < 0.30f) {
      in.tof_front_m = dist_to_obs1;
    } else if (dist_to_obs2 > 0.02f && dist_to_obs2 < 0.30f) {
      in.tof_front_m = dist_to_obs2;
    } else {
      in.tof_front_m = 1.0f;
    }

    const float centroid_mm = std::clamp(-pose.y_m * 1000.0f, -31.5f, 31.5f);
    const uint8_t prev_bypass = bypass_state;
    follower.update(pose, 0.45f, 0.45f * kDt, centroid_mm, true, kDt, in, out, target_v,
                    target_steer, waypoint_idx, bypass_state);

    if (prev_bypass == 0 && bypass_state == 1) {
      evasions_triggered++;
    }

    const float max_speed = prof_cfg.max_straight_speed_mps;
    const float vl = out.left_pwm * max_speed;
    const float vr = out.right_pwm * max_speed;
    const float v = 0.5f * (vl + vr);
    const float w = (vl - vr) / kWheelBase;

    pose.x_m += v * std::cos(pose.heading_rad) * kDt;
    pose.y_m += v * std::sin(pose.heading_rad) * kDt;
    pose.heading_rad += w * kDt;
  }

  // Must have triggered both evasions successfully in sequence
  assert(evasions_triggered == 2);
  assert(bypass_state == 0);
  assert(std::abs(pose.y_m) < 0.040f);

  std::cout << "[TEST] Sequential Obstacle Evasion: Successfully evaded " << evasions_triggered
            << " consecutive obstacles with smooth centerline re-lock\n";
}

void test_sat_obb_collision_detection() {
  // Case 1: Two axis-aligned separated boxes
  const lf::BoundingBox2D box_a{0.0f, 0.0f, 50.0f, 30.0f, 0.0f};
  const lf::BoundingBox2D box_b{120.0f, 0.0f, 50.0f, 30.0f, 0.0f};
  assert(!lf::check_obb_intersection(box_a, box_b));

  // Case 2: Two axis-aligned overlapping boxes
  const lf::BoundingBox2D box_c{90.0f, 0.0f, 50.0f, 30.0f, 0.0f};
  assert(lf::check_obb_intersection(box_a, box_c));

  // Case 3: Separated along Y axis
  const lf::BoundingBox2D box_d{0.0f, 100.0f, 50.0f, 30.0f, 0.0f};
  assert(!lf::check_obb_intersection(box_a, box_d));

  // Case 4: Rotated 45-degree box separated from axis-aligned box
  constexpr float kPi4 = kPi / 4.0f;
  const lf::BoundingBox2D box_rot_sep{150.0f, 0.0f, 40.0f, 20.0f, kPi4};
  assert(!lf::check_obb_intersection(box_a, box_rot_sep));

  // Case 5: Rotated 45-degree box overlapping corner of box_a
  const lf::BoundingBox2D box_rot_overlap{70.0f, 30.0f, 40.0f, 20.0f, kPi4};
  assert(lf::check_obb_intersection(box_a, box_rot_overlap));

  // Case 6: Fully nested/concentric boxes
  const lf::BoundingBox2D box_nested{0.0f, 0.0f, 20.0f, 10.0f, 0.2f};
  assert(lf::check_obb_intersection(box_a, box_nested));

  std::cout
    << "[TEST] SAT OBB 2D Collision Detection: All separation/intersection cases verified.\n";
}

void test_robot_obstacle_collision_and_immediate_halt() {
  const lf::RobotConfig config{};
  lf::RobotCoordinator robot(config);
  robot.init();
  robot.set_mode(lf::RobotMode::kRun1Map);

  assert(!robot.is_crashed());
  assert(!robot.get_telemetry().collision);

  // Obstacle at (500, 300) mm, 105x60 mm, 0 deg
  std::vector<lf::ObstacleGeometry> obstacles;
  obstacles.push_back({500.0f, 300.0f, 105.0f, 60.0f, 0.0f});

  constexpr float kRobotLength = 145.0f;
  constexpr float kRobotWidth = 138.0f;

  // 1. Robot far away -> No collision
  int hit =
    lf::check_robot_collision_any(200.0f, 300.0f, 0.0f, kRobotLength, kRobotWidth, obstacles);
  assert(hit == -1);

  // 2. Robot impacts obstacle -> Collision detected at index 0
  hit = lf::check_robot_collision_any(450.0f, 300.0f, 0.0f, kRobotLength, kRobotWidth, obstacles);
  assert(hit == 0);

  // 3. Report collision to coordinator -> Immediate emergency halt and fail
  robot.report_collision(hit);
  assert(robot.is_crashed());
  assert(robot.get_telemetry().collision);
  assert(robot.get_telemetry().obstacle.collision);
  assert(robot.get_mode() == lf::RobotMode::kIdle);

  // 4. In update loop after crash -> Motors actively braked (0 PWM, active_brake = true)
  lf::SensorData in{};
  in.dt = 0.002f;
  lf::MotorCmd out{};
  robot.update(in, out);
  assert(out.left_pwm == 0.0f);
  assert(out.right_pwm == 0.0f);
  assert(out.active_brake);

  // 5. Switching mode resets crash state
  robot.set_mode(lf::RobotMode::kRun1Map);
  assert(!robot.is_crashed());
  assert(!robot.get_telemetry().collision);

  std::cout << "[TEST] Robot Obstacle Collision Detection & Immediate Halt: Verified.\n";
}

}  // namespace

int main() {
  std::cout << "=== KIEM THU MODULAR ROBOT ARCHITECTURE (SSoT) ===\n";
  test_sat_obb_collision_detection();
  test_robot_obstacle_collision_and_immediate_halt();
  test_global_trajectory_optimizer_qp();
  test_time_optimal_velocity_profiler();
  test_stanley_continuous_tracker();
  test_centroid_calculation();
  test_steering_direction();
  test_differential_kinematics();
  test_closed_loop_line_convergence();
  test_tactical_safety_guardrails();
  test_trajectory_speed_profiling_and_follower();
  test_profiler_live_reoptimization();
  test_reset_pose_preserves_map();
  test_launch_velocity_fix();
  test_trajectory_planner_obstacle_bypass();
  test_rich_telemetry_and_wheel_velocities();
  test_coordinator_global_pipeline_with_obstacle();
  test_telemetry_packet_serialization_and_checksum();
  test_run1_corner_deceleration_and_line_loss_latch();
  test_finish_zone_pull_in_and_sprint2_completion();
  test_speed_scaled_pid_attenuation();
  test_kinematic_curvature_mapping();
  test_reactive_sprint_braking_cap();
  test_sprint_corridor_and_late_apex();
  test_sensor_area_integration_model();
  test_closed_loop_obstacle_hyperplane_isolation();
  test_run1_reactive_obstacle_evasion();
  test_relay_auto_tuner();
  test_sequential_obstacle_evasion();
  std::cout << "=== TAT CA KIEM THU DA DAT (ALL TESTS PASSED) ===\n";
  return 0;
}
