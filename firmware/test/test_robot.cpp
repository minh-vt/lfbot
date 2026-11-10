#include <cmath>
#include <cstdlib>
#include <drive.hpp>
#include <iostream>
#include <limits>
#include <robot.hpp>
#include <type_traits>

namespace {

lf::Calibration calibrated(float wheelbase = 120.0F, float speed = 500.0F) {
  lf::Calibration calibration;
  calibration.wheelbase_mm = wheelbase;
  calibration.half_length_mm = 72.5F;
  calibration.half_width_mm = 69.0F;
  calibration.reference_speed_mm_s = speed;
  calibration.wheel_mm_per_tick = 1.0F;
  calibration.duty_speed_mm_s = speed;
  calibration.motor_response_s = 0.060F;
  calibration.drive_acceleration_mm_s2 = 2000.0F;
  calibration.brake_deceleration_mm_s2 = 4000.0F;
  calibration.ir_period_s = 0.002F;
  calibration.ir_spot_radius_mm = 4.0F;
  calibration.ir_white.fill(255.0F);
  calibration.ir_black.fill(0.0F);
  // Test placement only; no hardware calibration is claimed.
  for (std::size_t i = 0; i < lf::kIrSensorCount; ++i) {
    const float offset = static_cast<float>(i) - 3.5F;
    calibration.ir_position_mm[i] = {55.0F, offset * 8.0F};
  }
  calibration.tof_forward_mm = 72.5F;
  calibration.tof_range_mm = 2000.0F;
  calibration.tof_half_fov_rad = 0.22F;
  calibration.obstacle_radius_mm = 122.55F;
  return calibration;
}

// Fresh, straight-line packet with a valid range reading.
lf::SensorMeasurements line_packet(std::uint64_t time_us, std::uint32_t sequence) {
  lf::SensorMeasurements packet;
  packet.time_us = time_us;
  for (auto& sample : packet.ir) {
    sample = {200.0F, time_us, sequence, lf::ReadingStatus::kValid};
  }
  packet.ir[3] = {0.0F, time_us, sequence, lf::ReadingStatus::kValid};
  packet.ir[4] = {0.0F, time_us, sequence, lf::ReadingStatus::kValid};
  packet.tof[0] = {600.0F, time_us, sequence, lf::ReadingStatus::kValid};
  for (auto& counter : packet.wheel_counters) {
    counter = {sequence * 5U, time_us, sequence, lf::ReadingStatus::kValid};
  }
  return packet;
}

// Unlike assert, test checks remain active if an environment defines NDEBUG.
void require(bool condition, const char* message = "") {
  if (!condition) {
    std::cerr << "bootstrap requirement failed: " << message << '\n';
    std::abort();
  }
}

class BootstrapSession final : public lf::RobotSession {
public:
  void run(lf::Robot& robot) noexcept override {
    require(disable_calls == 1);
    require(robot.observe().status.phase == lf::RacePhase::kIdle);
    lf::SensorMeasurements measurements;
    measurements.time_us = 2000;
    measurements.ir[0] = {255.0F, 2000, 1, lf::ReadingStatus::kValid};
    measurements.ir[1] = {0.0F, 2000, 1, lf::ReadingStatus::kValid};
    measurements.tof[0] = {600.0F, 2000, 7, lf::ReadingStatus::kValid};
    measurements.wheel_counters[0] = {0xfffffffeU, 2000, 1, lf::ReadingStatus::kValid};
    measurements.wheel_counters[1] = {0xfffffffeU, 2000, 1, lf::ReadingStatus::kValid};
    const auto output = robot.drive(measurements, lf::RaceRequest::kStart);
    require(robot.observe().status.phase == lf::RacePhase::kRunning);
    require(robot.observe().status.runs_used == 1);
    require(robot.observe().status.reason == lf::StopReason::kNone);
    require(output.mode == lf::MotorMode::kDrive);
    require(std::abs(output.left_duty) <= 1.0F && std::abs(output.right_duty) <= 1.0F);
    require(robot.observe().ir[0].value == 0.0F);
    require(robot.observe().ir[1].value == 1.0F);
    require(robot.observe().tof[0].value == 5.0F);
    require(robot.observe().tof[0].sequence == 7);
    lf::SensorMeasurements next;
    next.time_us = 4000;
    next.tof[0] = {600.0F, 2000, 7, lf::ReadingStatus::kValid};
    next.wheel_counters[0] = {1U, 4000, 2, lf::ReadingStatus::kValid};
    next.wheel_counters[1] = {1U, 4000, 2, lf::ReadingStatus::kValid};
    (void)robot.drive(next, lf::RaceRequest::kNone);
    require(std::abs(robot.observe().tof[0].age - (0.002F / 0.24F)) < 1e-6F);
    require(std::abs(robot.observe().tof[0].value - 5.0F) < 1e-6F);
    next.time_us = 6000;
    next.ir[0] = {std::numeric_limits<float>::quiet_NaN(), 6000, 2, lf::ReadingStatus::kValid};
    next.ir[1] = {128.0F, 8000, 2, lf::ReadingStatus::kValid};
    next.tof[0] = {-1.0F, 6000, 8, lf::ReadingStatus::kValid};
    (void)robot.drive(next, lf::RaceRequest::kNone);
    require(robot.observe().ir[0].status == lf::ReadingStatus::kInvalid);
    require(robot.observe().ir[1].status == lf::ReadingStatus::kInvalid);
    require(robot.observe().tof[0].status == lf::ReadingStatus::kInvalid);
    (void)robot.drive(next, lf::RaceRequest::kStart);
    require(robot.observe().status.reason == lf::StopReason::kInvalidTime);
    next.time_us = 8000;
    require(robot.drive(next, lf::RaceRequest::kAbort).mode == lf::MotorMode::kCoast);
    require(robot.observe().status.reason == lf::StopReason::kAborted);
    next = {};
    (void)robot.drive(next, lf::RaceRequest::kReset);
    require(robot.observe().status.phase == lf::RacePhase::kIdle);
    require(robot.observe().status.runs_used == 0);
    next.time_us = 2000;
    (void)robot.drive(next, lf::RaceRequest::kStart);
    require(robot.observe().status.phase == lf::RacePhase::kRunning);
    ran = true;
  }
  void disable_motors() noexcept override { ++disable_calls; }

  int disable_calls = 0;
  bool ran = false;
};

class InvalidSession final : public lf::RobotSession {
public:
  void run(lf::Robot& robot) noexcept override {
    require(robot.observe().status.phase == lf::RacePhase::kFault);
    require(robot.observe().status.reason == lf::StopReason::kInvalidCalibration);
    const lf::SensorMeasurements measurements;
    const auto output = robot.drive(measurements, lf::RaceRequest::kStart);
    require(output.mode == lf::MotorMode::kCoast);
    require(robot.observe().status.phase == lf::RacePhase::kFault);
    require(robot.observe().status.reason == lf::StopReason::kInvalidCalibration);
    require(robot.observe().status.runs_used == 0);
  }
  void disable_motors() noexcept override { ++disable_calls; }
  int disable_calls = 0;
};

class InvalidTuningSession final : public lf::RobotSession {
public:
  void run(lf::Robot& robot) noexcept override {
    require(robot.observe().status.phase == lf::RacePhase::kFault);
    require(robot.observe().status.reason == lf::StopReason::kInvalidConfiguration);
    const lf::SensorMeasurements measurements;
    const auto output = robot.drive(measurements, lf::RaceRequest::kStart);
    require(output.mode == lf::MotorMode::kCoast, "an invalid plan never drives");
    require(robot.observe().status.reason == lf::StopReason::kInvalidConfiguration);
  }
  void disable_motors() noexcept override {}
};

class InterleavedSession final : public lf::RobotSession {
public:
  explicit InterleavedSession(lf::Robot& outer) noexcept : outer_(outer) {}

  void run(lf::Robot& inner) noexcept override {
    lf::SensorMeasurements measurements;
    measurements.time_us = 1000;
    const auto outer_output = outer_.drive(measurements, lf::RaceRequest::kNone);
    require(inner.observe().time_us == 0);
    measurements.time_us = 4000;
    const auto inner_output = inner.drive(measurements, lf::RaceRequest::kStart);
    require(outer_.observe().time_us == 1000);
    require(inner.observe().time_us == 4000);
    require(outer_output.mode == lf::MotorMode::kCoast);
    require(inner_output.mode == lf::MotorMode::kDrive);
    require(std::abs(inner_output.left_duty) <= 1.0F && std::abs(inner_output.right_duty) <= 1.0F);
  }
  void disable_motors() noexcept override {}

private:
  lf::Robot& outer_;
};

class NestedSession final : public lf::RobotSession {
public:
  void run(lf::Robot& robot) noexcept override {
    const auto before = robot.observe();
    BootstrapSession nested;
    lf::run_robot(calibrated(), nested);
    require(nested.ran && nested.disable_calls == 2);
    require(robot.observe().status.runs_used == before.status.runs_used);
    require(robot.observe().time_us == before.time_us);
    InterleavedSession interleaved(robot);
    lf::run_robot(calibrated(90.0F, 300.0F), interleaved);
    require(robot.observe().time_us == 1000);
    require(before.time_us == 0);
    lf::SensorMeasurements measurements;
    measurements.time_us = 2000;
    const auto output = robot.drive(measurements, lf::RaceRequest::kStart);
    require(std::abs(output.left_duty) <= 1.0F);
    require(robot.observe().status.phase == lf::RacePhase::kRunning);
  }
  void disable_motors() noexcept override {}
};

// The packet clock advances while every sensor keeps its acquisition time: a
// frozen sample is not new evidence and must not renew motor output.
class FrozenSensorsSession final : public lf::RobotSession {
public:
  void run(lf::Robot& robot) noexcept override {
    const auto frozen = line_packet(2000, 1);
    const auto started = robot.drive(frozen, lf::RaceRequest::kStart);
    require(robot.observe().status.phase == lf::RacePhase::kRunning);
    require(started.mode == lf::MotorMode::kDrive && started.left_duty > 0.1F);
    lf::MotorOutput output = started;
    for (std::uint64_t time_us = 4000; time_us <= 600000; time_us += 2000) {
      auto packet = frozen;
      packet.time_us = time_us;
      output = robot.drive(packet, lf::RaceRequest::kNone);
      if (time_us == 100000) {
        require(output.left_duty <= 0.2F, "stale line evidence must not cruise at full speed");
      }
    }
    require(robot.observe().status.phase == lf::RacePhase::kFault,
            "a permanently stale sample is a sensor fault");
    require(robot.observe().status.reason == lf::StopReason::kSensorFault);
    require(output.mode == lf::MotorMode::kCoast, "a faulted run coasts");
    require(output.deadline_us == 600000, "a disabled output expires at the packet time");
  }
  void disable_motors() noexcept override {}
};

// A missing range reading means the path ahead is unknown, not empty.
class MissingRangeSession final : public lf::RobotSession {
public:
  void run(lf::Robot& robot) noexcept override {
    auto first = line_packet(2000, 1);
    first.tof[0] = {};
    const auto started = robot.drive(first, lf::RaceRequest::kStart);
    require(robot.observe().status.phase == lf::RacePhase::kRunning);
    require(started.mode == lf::MotorMode::kDrive);
    require(started.left_duty <= 0.2F, "unknown forward range must not look like open track");
    auto next = line_packet(4000, 2);
    next.tof[0] = {};
    const auto output = robot.drive(next, lf::RaceRequest::kNone);
    require(robot.observe().status.phase == lf::RacePhase::kRunning);
    require(output.left_duty <= 0.2F, "unknown forward range keeps the crawl limit");
  }
  void disable_motors() noexcept override {}
};

void verify_counter_position_precision() {
  const auto calibration = calibrated();
  lf::Drive drive(calibration);
  lf::SensorReadings readings;
  auto fault = lf::StopReason::kNone;
  for (std::uint32_t i = 0; i <= 90000; ++i) {
    lf::SensorMeasurements packet;
    packet.time_us = 2000ULL * (static_cast<std::uint64_t>(i) + 1U);
    for (auto& counter : packet.wheel_counters) {
      counter = {i, packet.time_us, i, lf::ReadingStatus::kValid};
    }
    require(drive.read_sensors(packet, readings, fault));
  }
  const float expected = 90000.0F * calibration.wheel_mm_per_tick / calibration.wheelbase_mm;
  require(
    std::abs(readings.wheel_positions[0].value - expected) <
      calibration.wheel_mm_per_tick / calibration.wheelbase_mm,
    "180 seconds of exact encoder ticks cannot accumulate more than one tick of conversion error");
}

void verify_independent_counter_history() {
  const auto calibration = calibrated();
  lf::Drive drive(calibration);
  lf::SensorReadings readings;
  auto fault = lf::StopReason::kNone;
  auto packet = line_packet(2000, 1);
  require(drive.read_sensors(packet, readings, fault));
  packet = line_packet(4000, 2);
  packet.wheel_counters[1].status = lf::ReadingStatus::kMissing;
  require(drive.read_sensors(packet, readings, fault));
  packet = line_packet(6000, 3);
  require(drive.read_sensors(packet, readings, fault));
  const float expected = 10.0F / calibration.wheelbase_mm;
  require(std::abs(readings.wheel_positions[0].value - expected) < 1e-6F &&
            std::abs(readings.wheel_positions[1].value - expected) < 1e-6F,
          "a missing wheel channel cannot reset either cumulative encoder history");
}

void verify_no_return_status() {
  lf::Drive drive(calibrated());
  lf::SensorMeasurements packet;
  packet.time_us = 4000;
  packet.tof[0] = {2000.0F, 2000, 9, lf::ReadingStatus::kNoReturn};
  lf::SensorReadings readings;
  auto fault = lf::StopReason::kNone;
  require(drive.read_sensors(packet, readings, fault));
  require(readings.tof[0].status == lf::ReadingStatus::kNoReturn &&
            std::abs(readings.tof[0].value - 2000.0F / 120.0F) < 1e-5F &&
            std::abs(readings.tof[0].age - 0.002F / 0.24F) < 1e-6F && readings.tof[0].sequence == 9,
          "normalization preserves no-return identity, bound, age and status");
  packet.time_us = 6000;
  packet.tof[0].time_us = 8000;
  require(drive.read_sensors(packet, readings, fault));
  require(readings.tof[0].status == lf::ReadingStatus::kInvalid,
          "a no-return sample with a future timestamp is still invalid");
}

void verify_map_geometry_validation() {
  auto calibration = calibrated();
  calibration.line_width_mm = std::numeric_limits<float>::infinity();
  require(!lf::validate_calibration(calibration), "line geometry must be finite");
  calibration = calibrated();
  calibration.tof_error_mm = -1.0F;
  require(!lf::validate_calibration(calibration), "a range error bound cannot be negative");
  calibration = calibrated();
  calibration.wheelbase_mm = std::numeric_limits<float>::min();
  calibration.reference_speed_mm_s = calibration.wheelbase_mm;
  calibration.duty_speed_mm_s = calibration.wheelbase_mm;
  calibration.drive_acceleration_mm_s2 = calibration.wheelbase_mm;
  calibration.brake_deceleration_mm_s2 = calibration.wheelbase_mm;
  calibration.motor_response_s = 1.0F;
  calibration.half_length_mm = calibration.wheelbase_mm;
  calibration.half_width_mm = calibration.wheelbase_mm;
  calibration.wheel_mm_per_tick = calibration.wheelbase_mm;
  calibration.ir_spot_radius_mm = calibration.wheelbase_mm;
  calibration.tof_forward_mm = calibration.wheelbase_mm;
  calibration.tof_range_mm = calibration.wheelbase_mm;
  calibration.obstacle_radius_mm = calibration.wheelbase_mm;
  for (auto& position : calibration.ir_position_mm) {
    position = {calibration.wheelbase_mm, 0.0F};
  }
  require(!lf::validate_calibration(calibration), "finite physical inputs must normalize finitely");
  calibration.line_width_mm = calibration.wheelbase_mm;
  require(lf::validate_calibration(calibration),
          "the synthetic tiny-unit fixture is otherwise valid");
  calibration.tof_error_mm = std::numeric_limits<float>::max();
  require(!lf::validate_calibration(calibration), "normalized range uncertainty must not overflow");
  lf::RaceTuning tuning;
  tuning.robot.ir_noise_bound = 1.0F;
  require(!lf::validate_race_tuning(tuning), "full-scale unknown IR cannot support line mapping");
}

}  // namespace

namespace {

// A brick on the line blocks the line arc and hides the line behind it. Stopping
// at the first obstacle would end every run at the brick, so the robot must keep
// driving on a checked body-right arc instead of turning in place or coasting.
class BypassSession final : public lf::RobotSession {
public:
  void run(lf::Robot& robot) noexcept override {
    auto packet = line_packet(2000, 1);
    for (auto& sample : packet.ir) {
      sample.value = 255.0F;  // the brick hides the line
    }
    packet.tof[0] = {90.0F, 2000, 1, lf::ReadingStatus::kValid};
    const auto first = robot.drive(packet, lf::RaceRequest::kStart);
    require(first.mode == lf::MotorMode::kDrive, "a blocked line arc must not stop the robot");
    require(first.left_duty > first.right_duty, "the bypass goes round on the body-right side");
    std::uint32_t sequence = 1;
    float travelled_mm = 0.0F;
    for (int tick = 0; tick < 20; ++tick) {
      ++sequence;
      travelled_mm += 1.0F;
      const std::uint64_t time_us = 2000U + static_cast<std::uint64_t>(tick + 1) * 20000U;
      auto next = line_packet(time_us, sequence);
      for (auto& sample : next.ir) {
        sample.value = 255.0F;
      }
      next.tof[0] = {90.0F, time_us, sequence, lf::ReadingStatus::kValid};
      for (auto& counter : next.wheel_counters) {
        // 1 mm per packet, so the pose gauge advances at 50 mm/s and the brick keeps
        // its recorded place instead of being overtaken by a fictional odometer.
        counter = {
          static_cast<std::uint32_t>(travelled_mm),
          time_us,
          sequence,
          lf::ReadingStatus::kValid,
        };
      }
      const auto output = robot.drive(next, lf::RaceRequest::kNone);
      require(output.mode == lf::MotorMode::kDrive, "the bypass keeps driving, never coasts");
      require(output.left_duty + output.right_duty > 0.0F, "the bypass drives forward");
    }
    require(robot.observe().status.reason == lf::StopReason::kNone);
    require(robot.observe().status.phase == lf::RacePhase::kRunning);
  }
  void disable_motors() noexcept override {}
};

void verify_bypass_keeps_progress() {
  BypassSession bypass;
  lf::run_robot(calibrated(), bypass);
  std::cout << "bootstrap: a blocked line arc bypasses instead of stopping PASS\n";
}

}  // namespace

int main() {
  verify_bypass_keeps_progress();
  verify_map_geometry_validation();
  verify_counter_position_precision();
  verify_independent_counter_history();
  verify_no_return_status();
  static_assert(!std::is_copy_constructible_v<lf::Robot>);
  static_assert(!std::is_move_constructible_v<lf::Robot>);
  static_assert(!std::is_destructible_v<lf::Robot>);
  BootstrapSession session;
  lf::run_robot(calibrated(), session);
  require(session.ran && session.disable_calls == 2);
  for (const float length : {
         0.0F,
         -1.0F,
         std::numeric_limits<float>::quiet_NaN(),
         std::numeric_limits<float>::infinity(),
       }) {
    InvalidSession invalid;
    lf::run_robot(calibrated(length), invalid);
    require(invalid.disable_calls == 2);
  }
  InvalidSession invalid_speed;
  lf::run_robot(calibrated(120.0F, 0.0F), invalid_speed);
  require(invalid_speed.disable_calls == 2);
  NestedSession nested;
  lf::run_robot(calibrated(), nested);
  FrozenSensorsSession frozen;
  lf::run_robot(calibrated(), frozen);
  MissingRangeSession missing_range;
  lf::run_robot(calibrated(), missing_range);
  lf::RaceTuning invalid_tuning;
  invalid_tuning.track.clearance_margin = -1.0F;  // negative clearance is invalid
  InvalidTuningSession invalid_tuning_session;
  lf::run_robot(calibrated(), invalid_tuning_session, invalid_tuning);
  std::cout << "bootstrap: scope/disable/start/fault/calibration/stale/nested owners PASS\n";
}
