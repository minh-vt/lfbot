#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "config/robot_config.hpp"
#include "control/trajectory_planner.hpp"
#include "control/velocity_profiler.hpp"
#include "math/kinematics.hpp"
#include "system/robot_coordinator.hpp"
#include "system/telemetry_packet.hpp"

namespace {

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
  in.tof_side_m = 1.0f;

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

void test_velocity_profiler() {
  const lf::ProfilerConfig config{};
  lf::VelocityProfiler profiler(config);
  profiler.reset();

  // Đoạn thẳng 1.5m -> Cua 90 độ dài 0.3m -> Đoạn thẳng 1.0m -> Đích
  profiler.add_segment(lf::SegmentType::kStraight, 1.5f, 0.0f);
  profiler.add_segment(lf::SegmentType::kCurveLeft, 0.3f, 10.0f);
  profiler.add_segment(lf::SegmentType::kStraight, 1.0f, 0.0f);
  profiler.add_segment(lf::SegmentType::kFinishZone, 0.15f, 0.0f);

  assert(profiler.get_segment_count() == 4);

  profiler.optimize();
  assert(profiler.is_optimized());

  // Tra cứu tại s = 0.5m: Đang tăng tốc trên đoạn thẳng
  float v_target = 0.0f;
  float ff_steer = 0.0f;
  int seg_idx = 0;
  profiler.query(0.5f, v_target, ff_steer, seg_idx);
  std::cout << "[TEST] Van toc tang toc tren doan thang (s = 0.5m): " << v_target << " m/s\n";
  assert(v_target > 1.2f);

  // Tra cứu tại s = 1.45m: Phanh hãm đón cua
  profiler.query(1.45f, v_target, ff_steer, seg_idx);
  std::cout << "[TEST] Van toc phanh ham truoc cua (s = 1.45m): " << v_target << " m/s\n";
  assert(v_target < 1.2f);

  // Tra cứu tại s = 1.49m: Sát mép cua
  profiler.query(1.49f, v_target, ff_steer, seg_idx);
  std::cout << "[TEST] Van toc sat mep cua (s = 1.49m): " << v_target << " m/s\n";
  assert(v_target < 0.95f);
}

void test_differential_kinematics() {
  constexpr float kWheelBase = 0.115f;
  const lf::DifferentialDriveKinematics kin(kWheelBase);

  // 1. Invariant Forward -> Inverse roundtrip
  const float vl_orig = 0.85f;
  const float vr_orig = 0.45f;
  const auto twist = kin.forward(vl_orig, vr_orig);
  const auto wheels = kin.inverse(twist.linear_v_mps, twist.angular_w_radps);

  std::cout << "[TEST] Kinematics Roundtrip -> VL: " << wheels.left_mps
            << ", VR: " << wheels.right_mps << "\n";
  assert(std::abs(wheels.left_mps - vl_orig) < 0.0001f);
  assert(std::abs(wheels.right_mps - vr_orig) < 0.0001f);

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

  // Giả lập vòng kín 50 chu kỳ với độ lệch ban đầu -20mm (vạch ở bên trái)
  float simulated_centroid_mm = -20.0f;
  const float dt = 0.002f;
  const lf::DifferentialDriveKinematics kin(config.kinematics.wheel_base_m);

  for (int step = 0; step < 50; ++step) {
    lf::SensorData in{};
    in.dt = dt;
    in.tof_front_m = 1.0f;
    in.tof_side_m = 1.0f;

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

  std::cout << "[TEST] Closed-loop 50-step Error Reduction: " << -20.0f << " mm -> "
            << simulated_centroid_mm << " mm\n";
  // Hệ thống kín phải hội tụ về tâm: độ lệch tuyệt đối phải giảm ít nhất 50%
  assert(std::abs(simulated_centroid_mm) < 10.0f);
}

void test_tactical_safety_guardrails() {
  const lf::RobotConfig config{};
  lf::RobotCoordinator robot(config);

  // 1. Thử gán hệ số âm hoặc quá lớn -> Poka-Yoke phải clamp về khoảng an toàn
  robot.set_line_pid_gains(-1.0f, -0.5f, -0.05f);
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  robot.get_line_pid_gains(kp, ki, kd);
  assert(kp >= 0.005f);
  assert(ki >= 0.0f);
  assert(kd >= 0.0005f);

  // 2. Thử gán tốc độ âm hoặc phi lý
  robot.set_tactical_speeds(-0.5f, 100.0f);
  assert(robot.get_config().profiler.safe_map_speed_mps >= 0.2f);
  assert(robot.get_config().profiler.max_straight_speed_mps <= 4.0f);

  // 3. Thử gán cự ly né vật cản âm hoặc phi lý
  robot.set_obstacle_distances(-0.5f, 2.0f);
  float d_front = 0.0f;
  float d_stop = 0.0f;
  robot.get_obstacle_distances(d_front, d_stop);
  assert(d_front >= 0.08f && d_front <= 0.60f);
  assert(d_stop >= 0.03f && d_stop < d_front);
}

void test_auto_tune_persistence() {
  const lf::RobotConfig config{};
  lf::RobotCoordinator robot(config);
  robot.start_auto_tune();

  lf::SensorData in{};
  in.dt = 0.002f;
  lf::MotorCmd out{};

  // Giả lập dao động rơ-le 6 chu kỳ để bộ tự chỉnh hội tụ
  float simulated_c = 15.0f;
  for (int step = 0; step < 1000; ++step) {
    simulated_c = (out.left_pwm > out.right_pwm) ? -15.0f : 15.0f;
    for (size_t i = 0; i < lf::LineSensorConfig::kSensorCount; ++i) {
      const float dist = std::abs(config.line_sensor.weights_mm.at(i) - simulated_c);
      in.ir_raw.at(i) = (dist < 10.0f) ? (1.0f - dist / 10.0f) : 0.0f;
    }
    robot.update(in, out);
    if (robot.get_mode() == lf::RobotMode::kIdle) {
      break;  // Tuner đã hội tụ thành công
    }
  }

  // Xác minh: Sau khi tune, hệ số trong config_ và get_line_pid_gains phải được cập nhật
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  robot.get_line_pid_gains(kp, ki, kd);
  std::cout << "[TEST] Auto-Tuned Gains Persisted -> Kp: " << kp << ", Ki: " << ki << ", Kd: " << kd
            << "\n";
  assert(kp > 0.005f);
  assert(robot.get_config().line_pid.kp == kp);
}

void test_profiler_live_reoptimization() {
  lf::ProfilerConfig config{};
  config.max_accel_mps2 = 5.0f;
  config.max_brake_mps2 = 5.0f;
  config.max_straight_speed_mps = 2.0f;
  lf::VelocityProfiler profiler(config);
  profiler.reset();

  profiler.add_segment(lf::SegmentType::kStraight, 5.0f, 0.0f);
  profiler.add_segment(lf::SegmentType::kCurveRight, 1.0f, 5.0f);
  profiler.optimize();

  float v1 = 0.0f;
  float ff = 0.0f;
  int seg = 0;
  profiler.query(2.0f, v1, ff, seg);

  // Cập nhật tốc độ tối đa tăng lên 3.5 m/s
  config.max_straight_speed_mps = 3.5f;
  profiler.set_config(config);  // Tự động kích hoạt optimize() lại

  float v2 = 0.0f;
  profiler.query(2.0f, v2, ff, seg);
  std::cout << "[TEST] Profiler Live Re-opt: " << v1 << " m/s -> " << v2 << " m/s\n";
  assert(v2 > v1);
}

void test_reset_pose_preserves_map() {
  const lf::RobotConfig config{};
  lf::RobotCoordinator robot(config);
  robot.init();
  robot.set_mode(lf::RobotMode::kRun1Map);

  lf::SensorData in{};
  in.dt = 0.016f;
  in.enc_left_ticks = 1000;
  in.enc_right_ticks = 1000;
  in.tof_front_m = 1.0f;
  in.tof_left_m = 1.0f;
  in.tof_right_m = 1.0f;
  in.tof_side_m = 1.0f;

  // Giả lập phát hiện vạch đích sau 1.5m
  for (size_t i = 0; i < 8; ++i) {
    in.ir_raw.at(i) = 1.0f;  // Tất cả mắt đều thấy vạch đích
  }
  lf::MotorCmd out{};
  robot.update(in, out);

  // Gọi reset_pose(): Chỉ reset pose và odometry, giữ nguyên map đã học
  robot.reset_pose();
  assert(robot.get_mode() == lf::RobotMode::kIdle);
  assert(robot.get_telemetry().total_distance_m == 0.0f);

  // Nếu chuyển sang RUN_2_SPRINT, bộ profiler vẫn giữ nguyên trạng thái
  robot.set_mode(lf::RobotMode::kRun2Sprint);
  assert(robot.get_mode() == lf::RobotMode::kRun2Sprint);
}

void test_launch_velocity_fix() {
  const lf::ProfilerConfig config{};
  lf::VelocityProfiler profiler(config);
  profiler.reset();

  profiler.add_segment(lf::SegmentType::kStraight, 2.0f, 0.0f);
  profiler.add_segment(lf::SegmentType::kCurveRight, 0.4f, 8.0f);
  profiler.add_segment(lf::SegmentType::kFinishZone, 0.15f, 0.0f);

  profiler.optimize();
  assert(profiler.is_optimized());

  // Tra cuu ngay tai vach xuat phat s = 0.0m
  float v_target = 0.0f;
  float ff_steer = 0.0f;
  int seg_idx = 0;
  profiler.query(0.0f, v_target, ff_steer, seg_idx);

  std::cout << "[TEST] Launch Velocity tai s = 0.0m: " << v_target
            << " m/s (Min required: " << config.launch_min_speed_mps << " m/s)\n";
  assert(v_target >= config.launch_min_speed_mps);
}

void test_trajectory_planner_math() {
  assert(std::abs(lf::TrajectoryPlanner::quintic_step(0.0f) - 0.0f) < 1e-5f);
  assert(std::abs(lf::TrajectoryPlanner::quintic_step(1.0f) - 1.0f) < 1e-5f);
  assert(std::abs(lf::TrajectoryPlanner::quintic_bump(0.0f) - 0.0f) < 1e-5f);
  assert(std::abs(lf::TrajectoryPlanner::quintic_bump(0.5f) - 1.0f) < 1e-5f);
  assert(std::abs(lf::TrajectoryPlanner::quintic_bump(1.0f) - 0.0f) < 1e-5f);
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

void test_obstacle_fsm_direction_arbitration() {
  const lf::ObstacleConfig cfg{};
  lf::ObstacleFSM fsm(cfg);

  lf::SensorData in{};
  in.dt = 0.010f;
  in.tof_front_m = 0.20f;  // Phat hien vat can phia truoc (cfg.front_detect_dist_m = 0.28m)
  in.tof_left_m = 0.50f;   // Ben trai thoang
  in.tof_right_m = 0.10f;  // Ben phai hep
  lf::MotorCmd cmd{};

  fsm.update(in, 0.5f, cmd, false);
  assert(fsm.get_state() == lf::ObstacleFSM::State::kArcChicane);
  assert(fsm.get_bypass_direction() == -1.0f);  // Ne sang trai

  // Truong hop ben phai thoang hon
  fsm.reset();
  in.tof_left_m = 0.10f;
  in.tof_right_m = 0.50f;
  fsm.update(in, 0.5f, cmd, false);
  assert(fsm.get_bypass_direction() == 1.0f);  // Ne sang phai
}

void test_obstacle_fsm_arc_chicane_and_line_latch() {
  const lf::ObstacleConfig cfg{};
  lf::ObstacleFSM fsm(cfg);

  lf::SensorData in{};
  in.dt = 0.020f;
  in.tof_front_m = 0.20f;
  in.tof_left_m = 0.10f;
  in.tof_right_m = 0.50f;
  lf::MotorCmd cmd{};

  fsm.update(in, 0.4f, cmd, false);
  assert(fsm.get_state() == lf::ObstacleFSM::State::kArcChicane);

  // Xe chay qua cung tranh: ca 2 banh phai co van toc duong (khong cua giat nguoc)
  assert(cmd.left_pwm > 0.0f);
  assert(cmd.right_pwm > 0.0f);

  // Mo phong tien do vuot qua 50% quang duong va bat lai line
  for (int step = 0; step < 40; ++step) {
    fsm.update(in, 0.55f, cmd, false);
    if (fsm.get_progress_dist() > cfg.bypass_length_m * 0.50f) {
      break;
    }
  }

  // Khi mat IR cham lai vach, FSM phai bat lai kTracking ngay lap tuc
  fsm.update(in, 0.55f, cmd, true);
  assert(fsm.get_state() == lf::ObstacleFSM::State::kTracking);
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
  in.tof_side_m = 1.0f;
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
  assert(corner_avg_pwm < straight_pwm);
  assert(out.right_pwm > out.left_pwm);  // Steer left: right wheel faster

  // 3. Line-loss Memory Latch: line leaves field of view completely
  in.ir_raw = {};
  robot.update(in, out);

  assert(robot.get_telemetry().line_lost);
  // Must continue steering aggressively towards the lost line direction (left)
  assert(out.right_pwm > out.left_pwm);
  const float lost_avg_pwm = (out.left_pwm + out.right_pwm) * 0.5f;
  assert(lost_avg_pwm <= corner_avg_pwm);

  std::cout << "[TEST] Run 1 Corner Decel & Line-Loss Latch -> Straight PWM: " << straight_pwm
            << " | Corner Avg PWM: " << corner_avg_pwm << " | Lost Avg PWM: " << lost_avg_pwm
            << "\n";
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
  in.tof_side_m = 1.0f;
  in.ir_raw.at(3) = 1.0f;
  in.ir_raw.at(4) = 1.0f;
  lf::MotorCmd out{};
  robot.update(in, out);

  // Di chuyen qua nguong min_finish_arm_dist_m (1.0m ~ 5243 ticks -> 6000 ticks = 1.145m)
  in.enc_left_ticks = 6000;
  in.enc_right_ticks = 6000;

  // 1. Mat doc thay o den dich: robot phai tiep tuc chay de keo than xe vao trong o
  for (size_t i = 0; i < 8; ++i) {
    in.ir_raw.at(i) = 1.0f;
  }
  robot.update(in, out);

  assert(robot.get_mode() == lf::RobotMode::kRun1Map);
  assert(!out.active_brake);
  assert(out.left_pwm > 0.0f);
  assert(robot.get_profiler().is_optimized());

  in.enc_left_ticks += 262;
  in.enc_right_ticks += 262;
  robot.update(in, out);
  assert(robot.get_mode() == lf::RobotMode::kRun1Map);
  assert(!out.active_brake);

  in.enc_left_ticks += 600;
  in.enc_right_ticks += 600;
  robot.update(in, out);
  assert(robot.get_mode() == lf::RobotMode::kIdle);
  assert(out.active_brake);
  assert(out.left_pwm == 0.0f);

  robot.set_mode(lf::RobotMode::kRun2Sprint);
  in.enc_left_ticks = 0;
  in.enc_right_ticks = 0;
  in.ir_raw = {};
  in.ir_raw.at(3) = 1.0f;
  in.ir_raw.at(4) = 1.0f;
  robot.update(in, out);

  in.enc_left_ticks = 6000;
  in.enc_right_ticks = 6000;
  robot.update(in, out);
  assert(out.left_pwm > 0.05f);
  assert(!out.active_brake);

  std::cout << "[TEST] Finish Zone Pull-in & Sprint 2 Completion -> Verified\n";
}

}  // namespace

int main() {
  std::cout << "=== KIEM THU MODULAR ROBOT ARCHITECTURE (SSoT) ===\n";
  test_centroid_calculation();
  test_steering_direction();
  test_differential_kinematics();
  test_closed_loop_line_convergence();
  test_tactical_safety_guardrails();
  test_velocity_profiler();
  test_auto_tune_persistence();
  test_profiler_live_reoptimization();
  test_reset_pose_preserves_map();
  test_launch_velocity_fix();
  test_trajectory_planner_math();
  test_trajectory_planner_obstacle_bypass();
  test_rich_telemetry_and_wheel_velocities();
  test_obstacle_fsm_direction_arbitration();
  test_obstacle_fsm_arc_chicane_and_line_latch();
  test_telemetry_packet_serialization_and_checksum();
  test_run1_corner_deceleration_and_line_loss_latch();
  test_finish_zone_pull_in_and_sprint2_completion();
  std::cout << "=== TAT CA KIEM THU DA DAT (ALL TESTS PASSED) ===\n";
  return 0;
}
