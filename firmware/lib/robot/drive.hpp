#pragma once

#include <array>
#include <cstdint>

#include "robot_data.hpp"

namespace lf {

// Turns one acquisition packet into normalized readings and a normalized motor
// demand into an output with a physical deadline. Owns sample history only; the
// decision about where to drive belongs to Race or to a learned policy.
class Drive final {
public:
  explicit Drive(const Calibration& calibration) noexcept;

  [[nodiscard]] bool calibration_valid() const noexcept { return calibration_valid_; }

  // Clears the monotonic sample history so the next packet starts a new session.
  void reset_session() noexcept;

  // Re-establishes the wheel-counter baseline after the world was repositioned.
  // The session clock and accumulated wheel travel stay intact.
  void prepare_run() noexcept;

  // Fails on non-monotonic time or invalid calibration; readings stay empty then.
  [[nodiscard]] bool read_sensors(const SensorMeasurements& measurements, SensorReadings& readings,
                                  StopReason& fault) noexcept;

  // A demand is applied only while driving is allowed; impossible demands are
  // reported instead of being clamped into a plausible-looking command.
  [[nodiscard]] MotorOutput make_output(const MotorDemand& demand, std::uint64_t now_us,
                                        bool allowed_to_drive, StopReason& fault) const noexcept;

private:
  Calibration calibration_;
  bool calibration_valid_;
  std::uint64_t previous_time_us_ = 0;
  bool has_previous_time_ = false;
  std::array<std::uint32_t, 2> previous_counters_{};
  std::array<double, 2> wheel_positions_{};
  std::array<bool, 2> has_counters_{};
};

}  // namespace lf
