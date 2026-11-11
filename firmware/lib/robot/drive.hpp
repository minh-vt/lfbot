#pragma once

#include <array>

#include "config.hpp"
#include "motion.hpp"
#include "types.hpp"

namespace lf {

// Forward speed (counts/s) and turn rate (rad/s, positive = left) in, one duty
// per wheel out. Each wheel runs its own speed loop: the motor model supplies
// the duty that should hold the target, a PI trims what the model gets wrong.
class Drive {
public:
  void reset();
  [[nodiscard]] MotorOutput step(const Motion& motion, float v_cps, float omega_rad_s,
                                 const Sample<float>& battery);
  [[nodiscard]] bool stalled() const;
  [[nodiscard]] const std::array<float, kWheelCount>& target_cps() const { return target_cps_; }

private:
  std::array<float, kWheelCount> target_cps_{};
  std::array<float, kWheelCount> integral_{};
  std::array<std::uint32_t, kWheelCount> stall_ticks_{};
};

}  // namespace lf
