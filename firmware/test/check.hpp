#pragma once

#include <cstdio>
#include <cstdlib>

// Tests prove contracts only; behaviour on the robot is proved by its logs.
inline void require(bool condition, const char* message) {
  if (!condition) {
    static_cast<void>(std::fprintf(stderr, "FAIL: %s\n", message));
    std::exit(1);
  }
}
