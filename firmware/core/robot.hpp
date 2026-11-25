#pragma once

#include <array>
#include <cstdint>

#include "course.hpp"
#include "drive.hpp"
#include "log.hpp"
#include "motion.hpp"
#include "pilot.hpp"
#include "types.hpp"

namespace lf {

// What other tasks may know about the robot, copied out once per tick.
struct Snapshot {
  Phase phase = Phase::kIdle;
  StopReason reason = StopReason::kNone;
  Guide guide = Guide::kLine;
  bool ready = false;
  int run = 1;  // the run going now, or the next one if none is
  std::uint32_t course_stretches = 0;
  float s_mm = 0.0F;
  float psi_deg = 0.0F;
  float yaw_rate_rad_s = 0.0F;
  float gyro_bias_rad_s = 0.0F;
  float battery_v = 0.0F;
  std::array<float, kWheelCount> cps{};
  std::size_t usable_eyes = 0;
  bool line_reads_higher = false;
  std::array<float, kIrCount> floor_level{};
  std::array<float, kIrCount> line_level{};
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
                             std::uint64_t now_us);
  [[nodiscard]] Snapshot snapshot() const;
  [[nodiscard]] const LogRing& log() const { return log_; }
  // Run 1 as kept in flash, given back at power-up before the first step; the next
  // race is then run 2. False if the record is unusable.
  [[nodiscard]] bool load_course(const CourseRecord& record) { return course_.load(record); }
  // Read by the platform to keep in flash, only while the robot is stopped.
  [[nodiscard]] const CourseRecord& course() const { return course_.record(); }

private:
  [[nodiscard]] bool detect_press(bool button_down, std::uint64_t now_us);
  [[nodiscard]] bool is_ready(const Measurements& measurements, std::uint64_t now_us) const;
  [[nodiscard]] StopReason check_safety(const Measurements& measurements,
                                        std::uint64_t now_us) const;
  void start(std::uint64_t now_us);
  void stop(StopReason reason);
  void settle(std::uint64_t now_us);
  [[nodiscard]] Target race(const Measurements& measurements, std::uint64_t now_us);
  [[nodiscard]] MotorOutput move(const Measurements& measurements, std::uint64_t now_us);
  [[nodiscard]] MotorOutput handle_phase(bool pressed, const Measurements& measurements,
                                         std::uint64_t now_us);
  [[nodiscard]] Colour led() const;
  void record(const Measurements& measurements, const MotorOutput& motor);

  Motion motion_;
  Drive drive_;
  Pilot pilot_;
  Course course_;
  LogRing log_;

  Phase phase_ = Phase::kIdle;
  StopReason reason_ = StopReason::kNone;
  bool ready_ = false;
  float battery_v_ = 0.0F;
  float pace_ = 1.0F;
  std::uint64_t phase_start_us_ = 0;
  std::uint32_t ir_missing_ticks_ = 0;

  bool button_raw_ = false;
  bool button_stable_ = false;
  std::uint64_t button_changed_us_ = 0;

  std::uint32_t log_ticks_ = 0;
  bool log_now_ = false;
};

}  // namespace lf
