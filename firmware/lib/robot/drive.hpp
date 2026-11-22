#pragma once

#include <cstdint>

#include "config.hpp"
#include "motion.hpp"
#include "types.hpp"

namespace lf {

// Applies a forward and a turn demand straight to the two wheels. There is no
// inner speed loop: the line is followed by duty directly, so a steering change
// reaches the wheels on the next tick instead of after the wheel-speed loop's lag.
class Drive {
public:
  void reset();
  [[nodiscard]] MotorOutput step(const Target& target, const Sample<float>& battery,
                                 const Motion& motion);
  [[nodiscard]] bool stalled() const { return stalled_; }

private:
  bool stalled_ = false;
  std::uint32_t stall_ticks_ = 0;
};

}  // namespace lf
