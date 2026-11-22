#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "check.hpp"
#include "config.hpp"
#include "line.hpp"
#include "suites.hpp"

namespace {

using Ir = std::array<std::uint16_t, lf::kIrCount>;

// Feeds the scan what a swing over a line produces: mostly floor, some line.
lf::LineCalibration scan_of(float floor_level, float line_level, std::size_t blind_eyes = 0) {
  lf::LineScan scan;
  for (int sample = 0; sample < 1500; ++sample) {
    Ir raw{};
    for (std::size_t eye = 0; eye < lf::kIrCount; ++eye) {
      const bool over_line = (sample + 37 * static_cast<int>(eye)) % 100 < 12;
      float level = over_line ? line_level : floor_level;
      level += static_cast<float>((sample * 7 + static_cast<int>(eye) * 3) % 13) - 6.0F;
      if (eye < blind_eyes) {
        level = 4095.0F;  // saturated: floor and line look the same
      }
      raw[eye] = static_cast<std::uint16_t>(level);
    }
    scan.add(raw);
  }
  return scan.calibration();
}

void test_scan_learns_either_polarity() {
  const lf::LineCalibration dark_line = scan_of(3300.0F, 700.0F);
  require(dark_line.valid() && dark_line.usable_count == lf::kIrCount, "a normal floor scans");
  require(!dark_line.line_reads_higher, "a line that reads lower is found to read lower");
  const lf::LineCalibration bright_line = scan_of(700.0F, 3300.0F);
  require(bright_line.valid() && bright_line.line_reads_higher,
          "and one that reads higher is found to read higher");
}

void test_scan_copes_with_a_bright_low_contrast_floor() {
  const lf::LineCalibration sunny = scan_of(3900.0F, 3300.0F);
  require(sunny.valid() && !sunny.line_reads_higher, "a sunlit floor with a pale line still scans");
  require(std::abs(sunny.floor[0] - 3900.0F) < 60.0F, "and the floor level is the common reading");
}

void test_blind_eyes_are_left_out_not_trusted() {
  const lf::LineCalibration two_blind = scan_of(3300.0F, 700.0F, 2);
  require(two_blind.valid() && two_blind.usable_count == 6, "two saturated eyes leave six");
  require(!two_blind.usable[0] && !two_blind.usable[1], "and they are the saturated ones");
  const lf::LineCalibration four_blind = scan_of(3300.0F, 700.0F, 4);
  require(!four_blind.valid(), "four saturated eyes are too few to follow a line");
}

void test_a_floor_near_the_ceiling_still_works_while_the_contrast_is_real() {
  // The floor leaves little headroom below the rail and the line clips at it.
  const lf::LineCalibration dark_floor = scan_of(3800.0F, 4095.0F);
  require(dark_floor.valid() && dark_floor.line_reads_higher,
          "a 300 count contrast over a quiet floor is a usable line");
  lf::LineScan wandering;
  for (int sample = 0; sample < 1500; ++sample) {
    Ir raw{};
    for (std::size_t eye = 0; eye < lf::kIrCount; ++eye) {
      // The floor itself wanders by +-150, as large as the line contrast.
      const float wander =
        static_cast<float>((sample * 31 + static_cast<int>(eye) * 17) % 301) - 150.0F;
      raw[eye] = static_cast<std::uint16_t>(3800.0F + wander);
    }
    wandering.add(raw);
  }
  require(!wandering.calibration().valid(),
          "but a floor that wanders as much as the contrast is not");
}

void test_glare_in_one_eye_does_not_flip_the_polarity() {
  // Real outdoor scan: one eye dipped to ~600 for a few samples while the line read
  // high everywhere. A vote that added raw tail lengths let that eye win.
  lf::LineScan scan;
  for (int sample = 0; sample < 1500; ++sample) {
    Ir raw{};
    for (std::size_t eye = 0; eye < lf::kIrCount; ++eye) {
      const bool over_line = (sample + 37 * static_cast<int>(eye)) % 100 < 12;
      float level = over_line ? 4095.0F : 3700.0F;
      level += static_cast<float>((sample * 7 + static_cast<int>(eye) * 3) % 13) - 6.0F;
      if (eye == 7 && sample % 40 == 0) {
        level = 600.0F;  // glare
      }
      raw[eye] = static_cast<std::uint16_t>(level);
    }
    scan.add(raw);
  }
  const lf::LineCalibration calibration = scan.calibration();
  require(calibration.line_reads_higher, "seven eyes seeing the line high outvote one eye's glare");
  require(calibration.valid(), "and the scan is usable");
}

void test_a_scan_that_never_crossed_the_line_is_refused() {
  require(!scan_of(3300.0F, 3300.0F).valid(), "no contrast anywhere means no calibration");
}

std::array<float, lf::kIrCount> dark_on(std::initializer_list<std::size_t> eyes) {
  std::array<float, lf::kIrCount> dark{};
  for (const std::size_t eye : eyes) {
    dark[eye] = 1.0F;
  }
  return dark;
}

void test_position_is_signed_left_to_right() {
  const lf::LineView centre = lf::read_line(dark_on({3, 4}), 0.0F);
  require(centre.shape == lf::LineShape::kLine && std::abs(centre.position) < 0.01F,
          "centre reads zero");
  const lf::LineView left = lf::read_line(dark_on({0, 1}), 0.0F);
  require(left.position < -2.0F, "D1 and D2 are the far left");
  const lf::LineView right = lf::read_line(dark_on({6, 7}), 0.0F);
  require(right.position > 2.0F, "D7 and D8 are the far right");
}

void test_shapes() {
  require(lf::read_line(dark_on({}), 0.0F).shape == lf::LineShape::kNone, "no dark eyes: none");
  require(lf::read_line(dark_on({1, 6}), 0.0F).shape == lf::LineShape::kSplit,
          "two patches: split");
  const lf::LineView wide = lf::read_line(dark_on({0, 1, 2, 3, 4, 5, 6, 7}), 0.0F);
  require(wide.shape == lf::LineShape::kWide, "most of the bar dark: a crossing or a dark area");
  require(wide.left_mass > 3.9F && wide.right_mass > 3.9F, "and the dark is on both halves");
  const lf::LineView corner = lf::read_line(dark_on({0, 1, 2, 3, 4, 5}), 0.0F);
  require(corner.shape == lf::LineShape::kWide && corner.left_mass > corner.right_mass,
          "more dark on the left half is counted as more on the left");
  // With two eyes lost, six dark is all of the bar.
  std::array<float, lf::kIrCount> six_usable = dark_on({1, 2, 3, 4, 5});
  six_usable[0] = -1.0F;
  six_usable[7] = -1.0F;
  six_usable[6] = 1.0F;
  require(lf::read_line(six_usable, 0.0F).shape == lf::LineShape::kWide,
          "the wide rule counts usable eyes, not eight");
}

void test_split_follows_the_patch_nearest_the_last_position() {
  const std::array<float, lf::kIrCount> dark = dark_on({1, 6});
  require(lf::read_line(dark, -2.0F).position < 0.0F, "stays on the left patch when it was left");
  require(lf::read_line(dark, 2.0F).position > 0.0F, "and on the right one when it was right");
}

void test_an_unusable_eye_is_ignored() {
  std::array<float, lf::kIrCount> dark = dark_on({3, 4});
  dark[0] = -1.0F;
  const lf::LineView view = lf::read_line(dark, 0.0F);
  require(view.shape == lf::LineShape::kLine && view.dark_eyes == 2,
          "an eye that is not used does not count");
}

void test_the_floor_level_follows_light_but_only_so_far() {
  lf::LineCalibration scanned = scan_of(3300.0F, 700.0F);
  lf::LineCalibration current = scanned;
  Ir shade{};
  shade.fill(3100);  // still clearly floor, a little darker than the scan found
  std::array<float, lf::kIrCount> dark = lf::darkness(current, shade);
  for (int tick = 0; tick < 5000; ++tick) {
    dark = lf::darkness(current, shade);
    lf::adapt_floor(current, scanned, shade, dark);
  }
  require(current.floor[3] < scanned.floor[3] - 100.0F, "the floor level moves with the light");
  const float limit = lf::kFloorAdaptLimit * std::abs(scanned.line[3] - scanned.floor[3]);
  require(current.floor[3] >= scanned.floor[3] - limit - 1.0F, "but never past its limit");

  lf::LineCalibration pinned = scanned;
  Ir on_line{};
  on_line.fill(800);  // the line, not floor
  for (int tick = 0; tick < 5000; ++tick) {
    lf::adapt_floor(pinned, scanned, on_line, lf::darkness(pinned, on_line));
  }
  require(std::abs(pinned.floor[3] - scanned.floor[3]) < 1.0F,
          "an eye on the line never drags the floor level");
}

}  // namespace

void run_line() {
  test_scan_learns_either_polarity();
  test_scan_copes_with_a_bright_low_contrast_floor();
  test_blind_eyes_are_left_out_not_trusted();
  test_a_floor_near_the_ceiling_still_works_while_the_contrast_is_real();
  test_glare_in_one_eye_does_not_flip_the_polarity();
  test_a_scan_that_never_crossed_the_line_is_refused();
  test_position_is_signed_left_to_right();
  test_shapes();
  test_split_follows_the_patch_nearest_the_last_position();
  test_an_unusable_eye_is_ignored();
  test_the_floor_level_follows_light_but_only_so_far();
  std::puts("line: PASS (levels, polarity, blind eyes, position, shapes, drift)");
}
