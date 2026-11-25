#include "robot.hpp"

#include <algorithm>
#include <cmath>

#include "config.hpp"

namespace lf {
namespace {

constexpr std::uint32_t kIrMissingTicks = 10;  // 20 ms without a new reading
constexpr float kDegreesPerRadian = 57.29577951F;

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
  const bool needs_ir = phase_ == Phase::kCalibrating || phase_ == Phase::kRacing;
  if (needs_ir && ir_missing_ticks_ >= kIrMissingTicks) {
    return StopReason::kIrStale;
  }
  if (motion_.gyro_disagrees()) {
    return StopReason::kGyroDisagrees;
  }
  if (drive_.stalled()) {
    return StopReason::kStall;
  }
  if (phase_ == Phase::kCalibrating && now_us - phase_start_us_ > kCalibrateTimeoutUs) {
    return StopReason::kScanFailed;
  }
  return StopReason::kNone;
}

void Robot::start(std::uint64_t now_us) {
  phase_ = Phase::kSettling;
  reason_ = StopReason::kNone;
  phase_start_us_ = now_us;
  motion_.begin_bias();
  log_now_ = true;
}

void Robot::stop(StopReason reason) {
  if (reason == StopReason::kFinish) {
    course_.finish();
  }
  phase_ = reason == StopReason::kFinish ? Phase::kFinished : Phase::kStopped;
  reason_ = reason;
  drive_.reset();
  log_now_ = true;
}

void Robot::settle(std::uint64_t now_us) {
  if (!motion_.update_bias(now_us)) {
    if (now_us - phase_start_us_ > kSettleTimeoutUs) {
      stop(StopReason::kNotStill);
    }
    return;
  }
  motion_.zero();
  drive_.reset();
  pilot_.begin(motion_);
  phase_ = Phase::kCalibrating;
  phase_start_us_ = now_us;
}

// Run 2 reads ahead in run 1's record for how fast to go; run 1 writes the record.
Target Robot::race(const Measurements& measurements, std::uint64_t now_us) {
  const float counts_per_tick =
    0.5F * (motion_.wheel_cps(0) + motion_.wheel_cps(1)) / static_cast<float>(kControlHz);
  pace_ = course_.pace(motion_.s_counts(), counts_per_tick);
  const Target target = pilot_.race(measurements, motion_, now_us, pace_);
  course_.note(motion_.s_counts(), target.turn, pilot_.guide());
  return target;
}

// Calibrating or racing: the pilot says where to go, unless something is unsafe.
MotorOutput Robot::move(const Measurements& measurements, std::uint64_t now_us) {
  const StopReason unsafe = check_safety(measurements, now_us);
  if (unsafe != StopReason::kNone) {
    stop(unsafe);
    return {};
  }
  const Target target = phase_ == Phase::kCalibrating ? pilot_.calibrate(measurements, motion_)
                                                      : race(measurements, now_us);
  if (pilot_.outcome() != StopReason::kNone) {
    stop(pilot_.outcome());
    return {};
  }
  if (phase_ == Phase::kCalibrating && pilot_.calibrated()) {
    // Wait here, motionless, until the operator presses again.
    phase_ = Phase::kReady;
    drive_.reset();
    log_now_ = true;
    return {};
  }
  return drive_.step(target, measurements.battery_v, motion_);
}

Colour Robot::led() const {
  switch (phase_) {
    case Phase::kIdle:
      if (!ready_) {
        return Colour::kOrange;
      }
      return course_.complete() ? Colour::kMagenta : Colour::kBlue;
    case Phase::kSettling:
    case Phase::kCalibrating:
      return Colour::kCyan;
    case Phase::kRacing:
      return Colour::kGreen;
    case Phase::kReady:
      return Colour::kPurple;
    case Phase::kStopped:
      return reason_ == StopReason::kButton ? Colour::kYellow : Colour::kRed;
    case Phase::kFinished:
      return Colour::kWhite;
  }
  return Colour::kRed;
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
  entry.guide = pilot_.guide();
  entry.shape = pilot_.view().shape;
  entry.dark_eyes = pilot_.view().dark_eyes;
  entry.position_tenths =
    static_cast<std::int8_t>(std::clamp(pilot_.view().position * 10.0F, -127.0F, 127.0F));
  entry.pace_percent = static_cast<std::uint8_t>(std::lround(pace_ * 100.0F));
  log_.push(entry);
}

MotorOutput Robot::handle_phase(bool pressed, const Measurements& measurements,
                                std::uint64_t now_us) {
  MotorOutput motor;
  switch (phase_) {
    case Phase::kIdle:
      if (pressed && ready_) {
        start(now_us);
      }
      break;
    case Phase::kSettling:
      // Stop beats everything, including the tick that would have done the work.
      if (pressed) {
        stop(StopReason::kButton);
      } else {
        settle(now_us);
      }
      break;
    case Phase::kCalibrating:
    case Phase::kRacing:
      if (pressed) {
        stop(StopReason::kButton);
      } else {
        motor = move(measurements, now_us);
      }
      break;
    case Phase::kReady:
      if (pressed) {
        pilot_.go();
        course_.start();
        pace_ = 1.0F;
        phase_ = Phase::kRacing;
        phase_start_us_ = now_us;
        log_now_ = true;
      }
      break;
    case Phase::kStopped:
    case Phase::kFinished:
      if (pressed) {
        phase_ = Phase::kIdle;  // the press acknowledges the stop
      }
      break;
  }
  // Ready holds the wheels so the robot cannot be knocked off its start; a stop holds
  // them so it does not roll on.
  if (phase_ == Phase::kReady || phase_ == Phase::kStopped || phase_ == Phase::kFinished) {
    motor = {};
    motor.mode = MotorMode::kBrake;
  }
  return motor;
}

Outputs Robot::step(const Measurements& measurements, bool button_down, std::uint64_t now_us) {
  motion_.update(measurements);
  ready_ = is_ready(measurements, now_us);
  battery_v_ =
    measurements.battery_v.status == SampleStatus::kValid ? measurements.battery_v.value : 0.0F;
  ir_missing_ticks_ = measurements.ir.status == SampleStatus::kValid ? 0 : ir_missing_ticks_ + 1;
  const bool pressed = detect_press(button_down, now_us);

  const MotorOutput motor = handle_phase(pressed, measurements, now_us);
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
  snapshot.guide = pilot_.guide();
  snapshot.ready = ready_;
  // Once racing, the run that is or was going; before that, the one a press would start.
  const bool raced =
    phase_ == Phase::kRacing || phase_ == Phase::kStopped || phase_ == Phase::kFinished;
  const bool run_2 = raced ? course_.replaying() : course_.complete();
  snapshot.run = run_2 ? 2 : 1;
  snapshot.course_stretches = course_.complete() ? course_.record().length : 0;
  snapshot.s_mm = motion_.s_counts() * kMmPerCount;
  snapshot.psi_deg = motion_.psi_rad() * kDegreesPerRadian;
  snapshot.yaw_rate_rad_s = motion_.yaw_rate_rad_s();
  snapshot.gyro_bias_rad_s = motion_.bias_rad_s();
  snapshot.battery_v = battery_v_;
  for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
    snapshot.cps[wheel] = motion_.wheel_cps(wheel);
  }
  snapshot.usable_eyes = pilot_.calibration().usable_count;
  snapshot.line_reads_higher = pilot_.calibration().line_reads_higher;
  snapshot.floor_level = pilot_.calibration().floor;
  snapshot.line_level = pilot_.calibration().line;
  return snapshot;
}

}  // namespace lf
