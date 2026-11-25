#pragma once

#include <cstdint>

#include "config.hpp"
#include "motion.hpp"
#include "types.hpp"

namespace lf {

// Mixes a forward and a turn demand into the two wheels' duty. There is no inner
// speed loop: a steering change reaches the wheels on the next tick. The turn is
// never clipped; forward gives way when a wheel would saturate, and forward changes
// by a bounded step per tick (ADR-024).
class Drive {
public:
  void reset();
  [[nodiscard]] MotorOutput step(const Target& target, const Sample<float>& battery,
                                 const Motion& motion);
  [[nodiscard]] bool stalled() const { return stalled_; }

private:
  float forward_ = 0.0F;  // what was applied last tick, after the step limit
  bool stalled_ = false;
  std::uint32_t stall_ticks_ = 0;
};

}  // namespace lf
