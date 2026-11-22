#include "robot.hpp"

#include <algorithm>
#include <cmath>

#include "config.hpp"

namespace lf {
namespace {

constexpr std::uint32_t kIrMissingTicks = 10;  // 20 ms without a new reading
constexpr float kDegreesPerRadian = 57.29577951F;
constexpr float kRadiansPerDegree = 0.01745329252F;

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
  if (pending_.kind == Command::Kind::kRace && ir_missing_ticks_ >= kIrMissingTicks) {
    return StopReason::kIrStale;
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

void Robot::stop(StopReason reason, std::uint64_t now_us) {
  if (reason == StopReason::kDone) {
    phase_ = Phase::kIdle;
    brake_until_us_ = now_us + kBrakeHoldUs;
  } else {
    phase_ = reason == StopReason::kFinish ? Phase::kFinished : Phase::kStopped;
    brake_until_us_ = 0;
  }
  reason_ = reason;
  drive_.reset();
  log_now_ = true;
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
  switch (pending_.kind) {
    case Command::Kind::kGo:
      maneuver_.begin_go(pending_.value, motion_, now_us);
      break;
    case Command::Kind::kTurn:
      maneuver_.begin_turn(pending_.value * kRadiansPerDegree, motion_, now_us);
      break;
    case Command::Kind::kRace:
    case Command::Kind::kNone:
      race_.begin(motion_, now_us);
      break;
  }
  phase_ = Phase::kRunning;
  phase_start_us_ = now_us;
  return {};
}

Target Robot::next_target(const Measurements& measurements, std::uint64_t now_us) {
  if (pending_.kind == Command::Kind::kRace) {
    const Target target = race_.step(measurements, motion_, now_us);
    if (race_.outcome() != StopReason::kNone) {
      stop(race_.outcome(), now_us);
    } else if (race_.state() == RaceState::kReady) {
      // Calibrated. Wait here, motionless, until the operator presses again.
      phase_ = Phase::kReady;
      drive_.reset();
      log_now_ = true;
    }
    return target;
  }
  const Target target = maneuver_.step(motion_, now_us);
  if (maneuver_.done()) {
    stop(StopReason::kDone, now_us);
  }
  return target;
}

MotorOutput Robot::run(const Measurements& measurements, std::uint64_t now_us) {
  MotorOutput brake;
  brake.mode = MotorMode::kBrake;
  const StopReason unsafe = check_safety(measurements, now_us);
  if (unsafe != StopReason::kNone) {
    stop(unsafe, now_us);
    return brake;
  }
  if (pending_.kind != Command::Kind::kRace && maneuver_.timed_out(now_us)) {
    stop(StopReason::kManeuverTimeout, now_us);
    return brake;
  }
  const Target target = next_target(measurements, now_us);
  if (phase_ != Phase::kRunning) {
    return {};  // it stopped, or finished calibrating: no wheel command this tick
  }
  return drive_.step(target, measurements.battery_v, motion_);
}

LedPattern Robot::led() const {
  switch (phase_) {
    case Phase::kIdle:
      return {ready_ ? Colour::kBlue : Colour::kOrange, Pattern::kSteady};
    case Phase::kSettling:
      return {Colour::kCyan, Pattern::kSteady};
    case Phase::kRunning: {
      const bool learning =
        pending_.kind == Command::Kind::kRace &&
        (race_.state() == RaceState::kScan || race_.state() == RaceState::kReturn);
      return {learning ? Colour::kCyan : Colour::kGreen, Pattern::kSteady};
    }
    case Phase::kReady:
      return {Colour::kPurple, Pattern::kSteady};
    case Phase::kStopped:
      return {reason_ == StopReason::kButton ? Colour::kYellow : Colour::kRed, Pattern::kSteady};
    case Phase::kFinished:
      return {Colour::kWhite, Pattern::kSteady};
  }
  return {Colour::kRed, Pattern::kSteady};
}

void Robot::record(const Measurements& measurements, const MotorOutput& motor) {
  if ((phase_ == Phase::kIdle || phase_ == Phase::kReady) && !log_now_) {
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
    entry.duty_percent[wheel] = static_cast<std::int8_t>(motor.duty[wheel] * 100.0F);
  }
  entry.ir = measurements.ir.value;
  entry.race = race_.state();
  entry.shape = race_.view().shape;
  entry.dark_eyes = race_.view().dark_eyes;
  entry.position_tenths =
    static_cast<std::int8_t>(std::clamp(race_.view().position * 10.0F, -127.0F, 127.0F));
  log_.push(entry);
}

MotorOutput Robot::handle_phase(bool pressed, const Measurements& measurements,
                                std::uint64_t now_us, const Command& command) {
  MotorOutput motor;
  switch (phase_) {
    case Phase::kIdle:
      if (command.kind != Command::Kind::kNone && ready_) {
        start(command, now_us);
      } else if (pressed && ready_) {
        start({Command::Kind::kRace, 0.0F}, now_us);
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
    case Phase::kReady:
      // Calibrated. A press sets it going; nothing else does.
      if (pressed) {
        race_.go();
        phase_ = Phase::kRunning;
        phase_start_us_ = now_us;
        log_now_ = true;
      }
      break;
    case Phase::kStopped:
    case Phase::kFinished:
      // A press acknowledges the stop; a console request is explicit enough to
      // start the next run directly.
      if (command.kind != Command::Kind::kNone && ready_) {
        start(command, now_us);
      } else if (pressed) {
        phase_ = Phase::kIdle;
      }
      break;
  }
  if (phase_ == Phase::kReady || phase_ == Phase::kStopped || phase_ == Phase::kFinished ||
      (phase_ == Phase::kIdle && now_us < brake_until_us_)) {
    // Ready holds the wheels braked so the robot cannot be knocked off its start.
    motor.mode = MotorMode::kBrake;
  }

  return motor;
}

Outputs Robot::step(const Measurements& measurements, bool button_down, std::uint64_t now_us,
                    const Command& command) {
  motion_.update(measurements);
  ready_ = is_ready(measurements, now_us);
  battery_v_ =
    measurements.battery_v.status == SampleStatus::kValid ? measurements.battery_v.value : 0.0F;
  ir_missing_ticks_ = measurements.ir.status == SampleStatus::kValid ? 0 : ir_missing_ticks_ + 1;
  const bool pressed = detect_press(button_down, now_us);

  const MotorOutput motor = handle_phase(pressed, measurements, now_us, command);
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
  snapshot.race = race_.state();
  snapshot.ready = ready_;
  snapshot.s_mm = motion_.s_counts() * kMmPerCount;
  snapshot.psi_deg = motion_.psi_rad() * kDegreesPerRadian;
  snapshot.yaw_rate_rad_s = motion_.yaw_rate_rad_s();
  snapshot.gyro_bias_rad_s = motion_.bias_rad_s();
  snapshot.battery_v = battery_v_;
  for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
    snapshot.cps[wheel] = motion_.wheel_cps(wheel);
  }
  snapshot.usable_eyes = race_.calibration().usable_count;
  snapshot.line_reads_higher = race_.calibration().line_reads_higher;
  snapshot.floor_level = race_.calibration().floor;
  snapshot.line_level = race_.calibration().line;
  return snapshot;
}

}  // namespace lf
