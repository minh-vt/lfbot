#pragma once

#include <esp_partition.h>

#include <cstddef>
#include <cstdint>

#include "log.hpp"

namespace lf::esp32 {

// The last few runs' logs in flash, one slot per run. The RAM log dies with the
// power; these are still there when the USB cable goes back in.
class Blackbox {
public:
  static constexpr std::size_t kSlots = 5;

  void initialize();
  // Slow (erases flash): call only while the robot is not moving.
  [[nodiscard]] bool save(const LogRing& log) const;
  // What a slot holds, for the console: false if it is empty or unreadable.
  [[nodiscard]] bool describe(std::size_t slot, std::uint32_t& sequence,
                              std::uint32_t& count) const;
  // run 0 is the newest, 1 the one before, and so on. Zero records if there is none.
  [[nodiscard]] std::size_t count(std::size_t run) const;
  [[nodiscard]] bool read(std::size_t run, std::size_t index, LogRecord& record) const;

private:
  struct Header {
    std::uint32_t magic = 0;
    std::uint32_t record_size = 0;
    std::uint32_t count = 0;
    std::uint32_t sequence = 0;
  };
  [[nodiscard]] bool read_header(std::size_t slot, Header& header) const;
  // The slot of the run `run` back from the newest, or kSlots if there is none.
  [[nodiscard]] std::size_t slot_of(std::size_t run) const;

  const esp_partition_t* partition_ = nullptr;
};

}  // namespace lf::esp32
