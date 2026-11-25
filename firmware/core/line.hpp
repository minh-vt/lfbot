#pragma once

#include <array>
#include <cstdint>

#include "config.hpp"
#include "types.hpp"

namespace lf {

// Each eye's floor and line levels as learned on this floor, under this light.
struct LineCalibration {
  std::array<float, kIrCount> floor{};
  std::array<float, kIrCount> line{};
  std::array<bool, kIrCount> usable{};
  bool line_reads_higher = false;
  std::size_t usable_count = 0;
  [[nodiscard]] bool valid() const { return usable_count >= kMinUsableEyes; }
};

// Readings collected while the robot swings over the line. The floor is what an
// eye sees most of the time; the line is the far tail on the rarer side. Neither
// the polarity nor any absolute level is assumed.
class LineScan {
public:
  void reset();
  void add(const std::array<std::uint16_t, kIrCount>& raw);
  [[nodiscard]] LineCalibration calibration() const;

private:
  static constexpr std::size_t kBins = 4096 / kScanBinWidth;
  [[nodiscard]] float level_at(std::size_t eye, float fraction) const;

  std::array<std::array<std::uint16_t, kBins>, kIrCount> histogram_{};
  std::uint32_t samples_ = 0;
};

struct LineView {
  LineShape shape = LineShape::kNone;
  // Centre of the tracked patch in eye pitches: -3.5 under D1 (far left),
  // +3.5 under D8 (far right).
  float position = 0.0F;
  std::uint8_t dark_eyes = 0;
  // How much dark there is on each half of the bar, for deciding which way a
  // junction turns off.
  float left_mass = 0.0F;
  float right_mass = 0.0F;
};

// How dark each usable eye is, 0 = floor, 1 = line; -1 for an eye not used.
std::array<float, kIrCount> darkness(const LineCalibration& calibration,
                                     const std::array<std::uint16_t, kIrCount>& raw);

// previous_position picks which patch to follow when there are two.
LineView read_line(const std::array<float, kIrCount>& dark, float previous_position);

// Light changes along the course (sun, shade). An eye that clearly sees floor
// pulls its floor level towards what it reads, by at most kFloorAdaptLimit of
// its contrast from the level the scan found.
void adapt_floor(LineCalibration& calibration, const LineCalibration& scanned,
                 const std::array<std::uint16_t, kIrCount>& raw,
                 const std::array<float, kIrCount>& dark);

}  // namespace lf
