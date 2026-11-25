#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "types.hpp"

namespace lf {

// One line of the black box. Raw enough to replay a failure from.
struct LogRecord {
  std::uint32_t t_ms = 0;
  Phase phase = Phase::kIdle;
  StopReason reason = StopReason::kNone;
  std::int16_t tof_mm = 0;
  std::uint8_t tof_status = 255;
  float s_counts = 0.0F;
  float psi_rad = 0.0F;
  float yaw_rate_rad_s = 0.0F;
  std::array<std::int16_t, kWheelCount> cps{};
  std::array<std::int8_t, kWheelCount> duty_percent{};
  std::array<std::uint16_t, kIrCount> ir{};
  Guide guide = Guide::kLine;
  LineShape shape = LineShape::kNone;
  std::uint8_t dark_eyes = 0;
  std::int8_t position_tenths = 0;
  std::uint8_t pace_percent = 100;  // run 2's speed-up over run 1
};

// Ring of the most recent records. The control task is the only writer; readers
// run when the robot is not running, so they never see a slot being overwritten.
class LogRing {
public:
  static constexpr std::size_t kCapacity = 4000;

  void push(const LogRecord& record) {
    const std::uint32_t total = total_.load(std::memory_order_relaxed);
    records_[total % kCapacity] = record;
    total_.store(total + 1, std::memory_order_release);
  }
  [[nodiscard]] std::size_t size() const {
    const std::uint32_t total = total_.load(std::memory_order_acquire);
    return total < kCapacity ? total : kCapacity;
  }
  // Index 0 is the oldest record still held.
  [[nodiscard]] const LogRecord& at(std::size_t index) const {
    const std::uint32_t total = total_.load(std::memory_order_acquire);
    return records_[(total - size() + index) % kCapacity];
  }
  void clear() { total_.store(0, std::memory_order_release); }

private:
  std::array<LogRecord, kCapacity> records_{};
  std::atomic<std::uint32_t> total_{0};
};

}  // namespace lf
