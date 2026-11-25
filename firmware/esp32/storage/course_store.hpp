#pragma once

#include <esp_partition.h>

#include "course.hpp"

namespace lf::esp32 {

// Run 1's record in flash, so run 2 survives a battery change. Written once, after a
// run 1 that finished; a record from different firmware or a torn write is refused.
class CourseStore {
public:
  void initialize();
  [[nodiscard]] bool load(CourseRecord& record) const;
  // Slow (erases flash): call only while the robot is not moving.
  [[nodiscard]] bool save(const CourseRecord& record) const;
  [[nodiscard]] bool erase() const;

private:
  const esp_partition_t* partition_ = nullptr;
};

}  // namespace lf::esp32
