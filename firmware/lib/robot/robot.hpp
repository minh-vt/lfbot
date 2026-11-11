#pragma once

#include <array>
#include <cstdint>

#include "drive.hpp"
#include "log.hpp"
#include "maneuver.hpp"
#include "motion.hpp"
#include "types.hpp"

namespace lf {

// A request from outside the control task (the bench console). `value` is
// millimetres for a go and degrees for a turn.
struct Command {
  enum class Kind : std::uint8_t { kNone, kGo, kTurn };
  Kind kind = Kind::kNone;
  float value = 0.0F;
};

// What other tasks may know about the robot, copied out once per tick.
struct Snapshot {
  Phase phase = Phase::kIdle;
  StopReason reason = StopReason::kNone;
  bool ready = false;
  bool routine_active = false;  // a bench routine is between or inside its steps
  float s_mm = 0.0F;
  float psi_deg = 0.0F;
  float yaw_rate_rad_s = 0.0F;
  float gyro_bias_rad_s = 0.0F;
  float battery_v = 0.0F;
  std::array<float, kWheelCount> cps{};
  std::array<float, kWheelCount> target_cps{};
};

// The only owner of robot state. One call per control tick; nothing else may
// touch it while the control task runs.
class Robot {
public:
  Robot() = default;
  Robot(const Robot&) = delete;
  Robot& operator=(const Robot&) = delete;
  Robot(Robot&&) = delete;
  Robot& operator=(Robot&&) = delete;
  ~Robot() = default;

  [[nodiscard]] Outputs step(const Measurements& measurements, bool button_down,
                             std::uint64_t now_us, const Command& command);
  [[nodiscard]] Snapshot snapshot() const;
  [[nodiscard]] const LogRing& log() const { return log_; }

private:
  [[nodiscard]] bool detect_press(bool button_down, std::uint64_t now_us);
  [[nodiscard]] bool is_ready(const Measurements& measurements, std::uint64_t now_us) const;
  [[nodiscard]] StopReason check_safety(const Measurements& measurements,
                                        std::uint64_t now_us) const;
  void start(const Command& command, std::uint64_t now_us);
  void start_script_step(std::uint64_t now_us);
  void stop(StopReason reason, std::uint64_t now_us);
  [[nodiscard]] MotorOutput settle(std::uint64_t now_us);
  [[nodiscard]] MotorOutput run(const Measurements& measurements, std::uint64_t now_us);
  [[nodiscard]] LedPattern led() const;
  void record(const Measurements& measurements, const MotorOutput& motor);

  Motion motion_;
  Drive drive_;
  Maneuver maneuver_;
  LogRing log_;

  Phase phase_ = Phase::kIdle;
  StopReason reason_ = StopReason::kNone;
  bool ready_ = false;
  float battery_v_ = 0.0F;
  Command pending_;
  std::uint64_t phase_start_us_ = 0;
  std::uint64_t brake_until_us_ = 0;

  bool button_raw_ = false;
  bool button_stable_ = false;
  std::uint64_t button_changed_us_ = 0;

  // Bench routine run by one button press, so the robot can be tested with the
  // USB cable out: the battery and the cable must not be connected together.
  int script_index_ = -1;
  std::uint64_t script_next_us_ = 0;

  std::uint32_t log_ticks_ = 0;
  bool log_now_ = false;
};

}  // namespace lf
