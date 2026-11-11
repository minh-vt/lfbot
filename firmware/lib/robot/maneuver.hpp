#pragma once

#include <cstdint>

#include "motion.hpp"

namespace lf {

struct Target {
  float v_cps = 0.0F;
  float omega_rad_s = 0.0F;
};

// One manoeuvre at a time: drive straight a distance, or turn through an angle.
// Progress comes from the wheels and the gyro; time is only the failure bound.
class Maneuver {
public:
  void begin_go(float mm, const Motion& motion, std::uint64_t now_us);
  void begin_turn(float rad, const Motion& motion, std::uint64_t now_us);
  [[nodiscard]] Target step(const Motion& motion, std::uint64_t now_us);
  [[nodiscard]] bool done() const { return done_; }
  [[nodiscard]] bool timed_out(std::uint64_t now_us) const;

private:
  enum class Kind : std::uint8_t { kGo, kTurn };
  [[nodiscard]] Target step_go(const Motion& motion, std::uint64_t now_us);
  [[nodiscard]] Target step_turn(const Motion& motion, std::uint64_t now_us);

  Kind kind_ = Kind::kGo;
  bool done_ = false;
  std::uint64_t start_us_ = 0;
  float goal_ = 0.0F;  // counts for a go, radians for a turn
  float s_start_ = 0.0F;
  float psi_start_ = 0.0F;
};

}  // namespace lf
