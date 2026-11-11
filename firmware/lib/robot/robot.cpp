#include "robot.hpp"

#include <algorithm>
#include <cmath>

#include "config.hpp"

namespace lf {
namespace {

constexpr float kDegreesPerRadian = 57.29577951F;
constexpr float kRadiansPerDegree = 0.01745329252F;

constexpr std::array<Command, 4> kBenchScript = {
  Command{Command::Kind::kGo, 300.0F},
  Command{Command::Kind::kTurn, 90.0F},
  Command{Command::Kind::kTurn, -90.0F},
  Command{Command::Kind::kGo, -300.0F},
};
constexpr std::uint64_t kScriptPauseUs = 700000;

std::int16_t to_int16(float value) {
  return static_cast<std::int16_t>(std::clamp(value, -32768.0F, 32767.0F));
}

}  // namespace

bool Robot::detect_press(bool button_down, std::uint64_t now_us) {
  if (button_down != button_raw_) {
    button_raw_ = button_down;
    button_changed_us_ = now_us;
  }
  if (button_raw_ == button_stable_ || now_us - button_changed_us_ < kButtonDebounceUs) {
    return false;
  }
  button_stable_ = button_raw_;
  return button_stable_;
}

bool Robot::is_ready(const Measurements& measurements, std::uint64_t now_us) const {
  return motion_.gyro_fresh(now_us) && measurements.encoders.status == SampleStatus::kValid &&
         measurements.battery_v.status == SampleStatus::kValid &&
         measurements.battery_v.value >= kMinBatteryV;
}

StopReason Robot::check_safety(const Measurements& measurements, std::uint64_t now_us) const {
  if (measurements.battery_v.status == SampleStatus::kValid &&
      measurements.battery_v.value < kMinBatteryV) {
    return StopReason::kBattery;
  }
  if (!motion_.gyro_fresh(now_us)) {
    return StopReason::kGyroStale;
  }
  if (motion_.gyro_disagrees()) {
    return StopReason::kGyroDisagrees;
  }
  if (drive_.stalled()) {
    return StopReason::kStall;
  }
  return StopReason::kNone;
}

void Robot::start(const Command& command, std::uint64_t now_us) {
  pending_ = command;
  phase_ = Phase::kSettling;
  reason_ = StopReason::kNone;
  phase_start_us_ = now_us;
  motion_.begin_bias();
  log_now_ = true;
}

void Robot::start_script_step(std::uint64_t now_us) {
  start(kBenchScript[static_cast<std::size_t>(script_index_)], now_us);
}

void Robot::stop(StopReason reason, std::uint64_t now_us) {
  phase_ = reason == StopReason::kDone ? Phase::kIdle : Phase::kStopped;
  brake_until_us_ = reason == StopReason::kDone ? now_us + kBrakeHoldUs : 0;
  reason_ = reason;
  drive_.reset();
  log_now_ = true;
  if (script_index_ < 0) {
    return;
  }
  // A finished step moves on to the next after a pause; anything else ends the routine.
  if (reason == StopReason::kDone &&
      static_cast<std::size_t>(script_index_) + 1 < kBenchScript.size()) {
    ++script_index_;
    script_next_us_ = now_us + kScriptPauseUs;
  } else {
    script_index_ = -1;
  }
}

MotorOutput Robot::settle(std::uint64_t now_us) {
  if (!motion_.update_bias(now_us)) {
    if (now_us - phase_start_us_ > kSettleTimeoutUs) {
      stop(StopReason::kNotStill, now_us);
    }
    return {};
  }
  motion_.zero();
  drive_.reset();
  if (pending_.kind == Command::Kind::kGo) {
    maneuver_.begin_go(pending_.value, motion_, now_us);
  } else {
    maneuver_.begin_turn(pending_.value * kRadiansPerDegree, motion_, now_us);
  }
  phase_ = Phase::kRunning;
  phase_start_us_ = now_us;
  return {};
}

MotorOutput Robot::run(const Measurements& measurements, std::uint64_t now_us) {
  MotorOutput brake;
  brake.mode = MotorMode::kBrake;
  const StopReason unsafe = check_safety(measurements, now_us);
  if (unsafe != StopReason::kNone) {
    stop(unsafe, now_us);
    return brake;
  }
  if (maneuver_.timed_out(now_us)) {
    stop(StopReason::kManeuverTimeout, now_us);
    return brake;
  }
  const Target target = maneuver_.step(motion_, now_us);
  if (maneuver_.done()) {
    stop(StopReason::kDone, now_us);
    return {};
  }
  return drive_.step(motion_, target.v_cps, target.omega_rad_s, measurements.battery_v);
}

LedPattern Robot::led() const {
  switch (phase_) {
    case Phase::kIdle:
      return {ready_ ? Colour::kBlue : Colour::kOrange, Pattern::kSteady};
    case Phase::kSettling:
      return {Colour::kCyan, Pattern::kSteady};
    case Phase::kRunning:
      return {Colour::kGreen, Pattern::kSteady};
    case Phase::kStopped:
      return {reason_ == StopReason::kButton ? Colour::kYellow : Colour::kRed, Pattern::kSteady};
  }
  return {Colour::kRed, Pattern::kSteady};
}

void Robot::record(const Measurements& measurements, const MotorOutput& motor) {
  if (phase_ == Phase::kIdle && !log_now_) {
    return;
  }
  if (++log_ticks_ < kLogEveryTicks && !log_now_) {
    return;
  }
  log_ticks_ = 0;
  log_now_ = false;
  LogRecord entry;
  entry.t_ms = static_cast<std::uint32_t>(measurements.encoders.latest_us / 1000);
  entry.phase = phase_;
  entry.reason = reason_;
  // A reading counts only if it is valid and recent; an old one is not "the
  // distance now", and logging it as such hid a stuck ToF for a whole run.
  const Sample<float>& distance = measurements.distance_mm;
  const bool tof_fresh = distance.status == SampleStatus::kValid &&
                         measurements.encoders.latest_us - distance.latest_us <= kTofStaleUs;
  entry.tof_mm = tof_fresh ? to_int16(distance.value) : static_cast<std::int16_t>(-1);
  entry.tof_status = measurements.range_status;
  entry.s_counts = motion_.s_counts();
  entry.psi_rad = motion_.psi_rad();
  entry.yaw_rate_rad_s = motion_.yaw_rate_rad_s();
  for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
    entry.cps[wheel] = to_int16(motion_.wheel_cps(wheel));
    entry.target_cps[wheel] = to_int16(drive_.target_cps()[wheel]);
    entry.duty_percent[wheel] = static_cast<std::int8_t>(motor.duty[wheel] * 100.0F);
  }
  entry.ir = measurements.ir.value;
  log_.push(entry);
}

Outputs Robot::step(const Measurements& measurements, bool button_down, std::uint64_t now_us,
                    const Command& command) {
  motion_.update(measurements);
  ready_ = is_ready(measurements, now_us);
  battery_v_ =
    measurements.battery_v.status == SampleStatus::kValid ? measurements.battery_v.value : 0.0F;
  const bool pressed = detect_press(button_down, now_us);

  MotorOutput motor;
  switch (phase_) {
    case Phase::kIdle:
      if (pressed && script_index_ >= 0) {
        stop(StopReason::kButton, now_us);  // a press between steps cancels the routine
      } else if (command.kind != Command::Kind::kNone && ready_) {
        start(command, now_us);
      } else if (pressed && ready_) {
        script_index_ = 0;
        start_script_step(now_us);
      } else if (script_index_ >= 0 && now_us >= script_next_us_) {
        start_script_step(now_us);
      }
      break;
    case Phase::kSettling:
      // Stop beats everything, including the tick that would have done the work.
      if (pressed) {
        stop(StopReason::kButton, now_us);
      } else {
        motor = settle(now_us);
      }
      break;
    case Phase::kRunning:
      if (pressed) {
        stop(StopReason::kButton, now_us);
        motor.mode = MotorMode::kBrake;
      } else {
        motor = run(measurements, now_us);
      }
      break;
    case Phase::kStopped:
      motor.mode = MotorMode::kBrake;
      if (pressed) {
        phase_ = Phase::kIdle;
        motor.mode = MotorMode::kCoast;
      }
      break;
  }
  if (phase_ == Phase::kStopped || (phase_ == Phase::kIdle && now_us < brake_until_us_)) {
    motor.mode = MotorMode::kBrake;
  }

  Outputs outputs;
  outputs.motor = motor;
  record(measurements, motor);
  // The wiring fact is applied here and nowhere else, so positive duty means
  // forward everywhere above this line.
  for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
    if (kInvertWheel[wheel]) {
      outputs.motor.duty[wheel] = -outputs.motor.duty[wheel];
    }
  }
  outputs.led = led();
  return outputs;
}

Snapshot Robot::snapshot() const {
  Snapshot snapshot;
  snapshot.phase = phase_;
  snapshot.reason = reason_;
  snapshot.ready = ready_;
  snapshot.routine_active = script_index_ >= 0;
  snapshot.s_mm = motion_.s_counts() * kMmPerCount;
  snapshot.psi_deg = motion_.psi_rad() * kDegreesPerRadian;
  snapshot.yaw_rate_rad_s = motion_.yaw_rate_rad_s();
  snapshot.gyro_bias_rad_s = motion_.bias_rad_s();
  snapshot.battery_v = battery_v_;
  for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
    snapshot.cps[wheel] = motion_.wheel_cps(wheel);
    snapshot.target_cps[wheel] = drive_.target_cps()[wheel];
  }
  return snapshot;
}

}  // namespace lf
