#include <cstdio>

#include "suites.hpp"

int main() {
  run_line();
  run_motion();
  run_robot();
  run_pilot();
  run_course();
  std::puts("all contracts: PASS (not a robot run)");
}
