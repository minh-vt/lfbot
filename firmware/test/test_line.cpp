#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "check.hpp"
#include "config.hpp"
#include "line_sensor.hpp"
#include "suites.hpp"

namespace {

using Ir = std::array<std::uint16_t, lf::kIrCount>;

Ir make(const std::array<float, lf::kIrCount>& values) {
  Ir raw{};
  for (std::size_t i = 0; i < lf::kIrCount; ++i) {
    raw[i] = static_cast<std::uint16_t>(values[i]);
  }
  return raw;
}

std::array<float, lf::kIrCount> filled(float value) {
  std::array<float, lf::kIrCount> values{};
  values.fill(value);
  return values;
}

// A working sensor bar: white reads high, the line reads low.
lf::LineCalibration calibration() {
  return lf::make_calibration(filled(2800.0F), filled(700.0F));
}

void test_centre_of_line_is_centred() {
  // Line under the middle two eyes only.
  std::array<float, lf::kIrCount> raw = filled(2800.0F);
  raw[3] = 700.0F;
  raw[4] = 700.0F;
  const lf::LineReading line = lf::read_line(calibration(), make(raw));
  require(line.valid, "a full calibration reads");
  require(line.on_line, "the line is seen");
  require(std::abs(line.position) < 0.05F, "the middle of the bar reports zero");
}

void test_position_follows_the_line() {
  std::array<float, lf::kIrCount> left = filled(2800.0F);
  left[1] = 700.0F;
  std::array<float, lf::kIrCount> right = filled(2800.0F);
  right[6] = 700.0F;
  const lf::LineReading l = lf::read_line(calibration(), make(left));
  const lf::LineReading r = lf::read_line(calibration(), make(right));
  require(l.position < -1.5F, "a line at the second eye is far left");
  require(r.position > 1.5F, "a line at the seventh eye is far right");
}

void test_white_is_not_a_line() {
  const lf::LineReading line = lf::read_line(calibration(), make(filled(2750.0F)));
  require(line.valid, "white still reads");
  require(!line.on_line, "white is not a line");
}

void test_an_eye_that_cannot_tell_white_from_black_refuses() {
  std::array<float, lf::kIrCount> black = filled(700.0F);
  black[5] = 2700.0F;  // this eye sees the same on both: dead or blocked
  const lf::LineCalibration bad = lf::make_calibration(filled(2800.0F), black);
  require(!bad.valid, "one blind eye invalidates the calibration");
  const lf::LineReading line = lf::read_line(bad, make(filled(2700.0F)));
  require(!line.valid, "and nothing is read from it");
}

}  // namespace

void run_line() {
  std::puts("line: PASS (position, white, blind eye)");
  test_centre_of_line_is_centred();
  test_position_follows_the_line();
  test_white_is_not_a_line();
  test_an_eye_that_cannot_tell_white_from_black_refuses();
}
