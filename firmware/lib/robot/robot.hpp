#pragma once

#include "robot_data.hpp"

namespace lf {

class RobotSession;

// Borrowed facade over a private, scoped owner. Robots are created only by
// run_robot and never copied, moved or heap-allocated by a platform.
class Robot final {
public:
  Robot(const Robot&) = delete;
  Robot& operator=(const Robot&) = delete;
  Robot(Robot&&) = delete;
  Robot& operator=(Robot&&) = delete;

  MotorOutput drive(const SensorMeasurements& measurements, RaceRequest request) noexcept;
  [[nodiscard]] const RobotObservation& observe() const noexcept;

  // Between runs: rebase wheel counters and per-run progress, keep memory.
  void prepare_run() noexcept;
  // After judging: stop driving; a later start may use the recorded memory.
  void finish_run() noexcept;

private:
  struct State;
  explicit Robot(State& state) noexcept : state_(state) {}
  ~Robot() = default;
  friend void run_robot(const Calibration& calibration, RobotSession& session,
                        const RaceTuning& tuning) noexcept;
  State& state_;
};

class RobotSession {
public:
  RobotSession() = default;
  RobotSession(const RobotSession&) = delete;
  RobotSession& operator=(const RobotSession&) = delete;
  RobotSession(RobotSession&&) = delete;
  RobotSession& operator=(RobotSession&&) = delete;
  virtual ~RobotSession() = default;
  virtual void run(Robot& robot) noexcept = 0;
  virtual void disable_motors() noexcept = 0;
};

// Robot and observation references must not outlive the callback.
// Normal-return disable does not handle callback stall/termination.
void run_robot(const Calibration& calibration, RobotSession& session,
               const RaceTuning& tuning = {}) noexcept;

}  // namespace lf
