#pragma once

#include <esp_partition.h>

#include <cstddef>

#include "log.hpp"

namespace lf::esp32 {

// The last run's log in flash. The RAM log dies with the power; this one is
// still there when the USB cable goes back in.
class Blackbox {
public:
  void initialize();
  // Slow (erases flash): call only while the robot is not moving.
  [[nodiscard]] bool save(const LogRing& log) const;
  [[nodiscard]] std::size_t count() const;
  [[nodiscard]] bool read(std::size_t index, LogRecord& record) const;

private:
  const esp_partition_t* partition_ = nullptr;
};

}  // namespace lf::esp32
