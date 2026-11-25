#include "line.hpp"

#include <algorithm>
#include <cmath>

namespace lf {
namespace {

constexpr float kAdcFullScale = 4095.0F;
constexpr float kCentre = 0.5F * static_cast<float>(kIrCount - 1);

float eye_position(std::size_t eye) {
  return static_cast<float>(eye) - kCentre;
}

}  // namespace

void LineScan::reset() {
  for (auto& eye : histogram_) {
    eye.fill(0);
  }
  samples_ = 0;
}

void LineScan::add(const std::array<std::uint16_t, kIrCount>& raw) {
  for (std::size_t eye = 0; eye < kIrCount; ++eye) {
    const std::size_t bin = std::min<std::size_t>(raw[eye] / kScanBinWidth, kBins - 1);
    std::uint16_t& count = histogram_[eye][bin];
    if (count < UINT16_MAX) {
      ++count;
    }
  }
  ++samples_;
}

// The reading below which `fraction` of this eye's samples fall.
float LineScan::level_at(std::size_t eye, float fraction) const {
  const auto wanted = static_cast<std::uint32_t>(fraction * static_cast<float>(samples_));
  std::uint32_t seen = 0;
  for (std::size_t bin = 0; bin < kBins; ++bin) {
    seen += histogram_[eye][bin];
    if (seen > wanted) {
      return (static_cast<float>(bin) + 0.5F) * static_cast<float>(kScanBinWidth);
    }
  }
  return kAdcFullScale;
}

LineCalibration LineScan::calibration() const {
  LineCalibration result;
  if (samples_ == 0) {
    return result;
  }
  std::array<float, kIrCount> median{};
  std::array<float, kIrCount> low{};
  std::array<float, kIrCount> high{};
  int higher_votes = 0;
  for (std::size_t eye = 0; eye < kIrCount; ++eye) {
    median[eye] = level_at(eye, 0.5F);
    low[eye] = level_at(eye, kScanTailFraction);
    high[eye] = level_at(eye, 1.0F - kScanTailFraction);
    // The line is the rare reading, so it sticks out further on its side. Each eye
    // votes once, by where the broader tails reach: glare that throws one eye far
    // off in a few samples must not outweigh the seven that saw the line.
    const float up = level_at(eye, 1.0F - kPolarityTailFraction) - median[eye];
    const float down = median[eye] - level_at(eye, kPolarityTailFraction);
    if (up > down) {
      ++higher_votes;
    } else if (down > up) {
      --higher_votes;
    }
  }
  // One polarity for the whole bar: the eyes are the same part on the same floor.
  result.line_reads_higher = higher_votes > 0;
  for (std::size_t eye = 0; eye < kIrCount; ++eye) {
    result.floor[eye] = median[eye];
    result.line[eye] = result.line_reads_higher ? high[eye] : low[eye];
    // How far the floor strays from its usual level on the side away from the line.
    const float floor_spread = result.line_reads_higher
                                 ? median[eye] - level_at(eye, kSpreadTailFraction)
                                 : level_at(eye, 1.0F - kSpreadTailFraction) - median[eye];
    const float separation = std::abs(result.line[eye] - result.floor[eye]);
    result.usable[eye] =
      separation >= kMinSeparationCounts && separation >= kContrastOverSpread * floor_spread;
    result.usable_count += result.usable[eye] ? 1 : 0;
  }
  return result;
}

std::array<float, kIrCount> darkness(const LineCalibration& calibration,
                                     const std::array<std::uint16_t, kIrCount>& raw) {
  std::array<float, kIrCount> dark{};
  for (std::size_t eye = 0; eye < kIrCount; ++eye) {
    if (!calibration.usable[eye]) {
      dark[eye] = -1.0F;
      continue;
    }
    const float span = calibration.line[eye] - calibration.floor[eye];
    dark[eye] =
      std::clamp((static_cast<float>(raw[eye]) - calibration.floor[eye]) / span, 0.0F, 1.0F);
  }
  return dark;
}

namespace {

// A patch is a run of eyes that clearly see the line. An unusable eye neither
// joins nor splits a run.
struct Run {
  std::size_t first = 0;
  std::size_t last = 0;
};

std::size_t find_runs(const std::array<float, kIrCount>& dark, std::array<Run, kIrCount>& runs,
                      std::uint8_t& dark_eyes) {
  std::size_t count = 0;
  bool inside = false;
  for (std::size_t eye = 0; eye < kIrCount; ++eye) {
    if (dark[eye] < 0.0F) {
      continue;
    }
    if (dark[eye] < kLineOnFraction) {
      inside = false;
      continue;
    }
    ++dark_eyes;
    if (!inside) {
      runs[count].first = eye;
      ++count;
      inside = true;
    }
    runs[count - 1].last = eye;
  }
  return count;
}

// A lone eye next to a real patch is a speck (an eye near the ADC rail flickering
// across its threshold), not a second line.
std::size_t drop_specks(std::array<Run, kIrCount>& runs, std::size_t count,
                        std::uint8_t& dark_eyes) {
  auto* const end = runs.data() + count;
  if (!std::any_of(runs.data(), end, [](const Run& run) { return run.last > run.first; })) {
    return count;
  }
  std::size_t kept = 0;
  for (std::size_t i = 0; i < count; ++i) {
    if (runs[i].last > runs[i].first) {
      runs[kept++] = runs[i];
    } else {
      --dark_eyes;
    }
  }
  return kept;
}

// The patch's centre: its dark eyes and the faint ones beside them.
float patch_centre(const std::array<float, kIrCount>& dark, const Run& run) {
  const std::size_t from = run.first > 0 ? run.first - 1 : 0;
  const std::size_t to = std::min(run.last + 1, kIrCount - 1);
  float weighted = 0.0F;
  float total = 0.0F;
  for (std::size_t eye = from; eye <= to; ++eye) {
    if (dark[eye] > 0.0F) {
      weighted += dark[eye] * eye_position(eye);
      total += dark[eye];
    }
  }
  return weighted / total;
}

}  // namespace

LineView read_line(const std::array<float, kIrCount>& dark, float previous_position) {
  LineView view;
  std::array<Run, kIrCount> runs{};
  std::size_t count = find_runs(dark, runs, view.dark_eyes);
  if (count == 0) {
    return view;
  }
  count = drop_specks(runs, count, view.dark_eyes);
  for (std::size_t eye = 0; eye < kIrCount; ++eye) {
    if (dark[eye] > 0.0F) {
      (eye < kIrCount / 2 ? view.left_mass : view.right_mass) += dark[eye];
    }
  }
  // With two patches, follow the one nearest where the line was.
  float best_distance = 1.0e9F;
  for (std::size_t i = 0; i < count; ++i) {
    const float centre = patch_centre(dark, runs[i]);
    if (std::abs(centre - previous_position) < best_distance) {
      best_distance = std::abs(centre - previous_position);
      view.position = centre;
    }
  }
  const auto usable = static_cast<float>(
    std::count_if(dark.begin(), dark.end(), [](float value) { return value >= 0.0F; }));
  if (static_cast<float>(view.dark_eyes) >= kWideFraction * usable) {
    view.shape = LineShape::kWide;
  } else {
    view.shape = count > 1 ? LineShape::kSplit : LineShape::kLine;
  }
  return view;
}

void adapt_floor(LineCalibration& calibration, const LineCalibration& scanned,
                 const std::array<std::uint16_t, kIrCount>& raw,
                 const std::array<float, kIrCount>& dark) {
  const float step = kFloorAdaptPerS * kControlDt;
  for (std::size_t eye = 0; eye < kIrCount; ++eye) {
    if (!calibration.usable[eye] || dark[eye] > kFloorSureFraction) {
      continue;
    }
    const float limit = kFloorAdaptLimit * std::abs(scanned.line[eye] - scanned.floor[eye]);
    const float moved =
      calibration.floor[eye] + step * (static_cast<float>(raw[eye]) - calibration.floor[eye]);
    calibration.floor[eye] =
      std::clamp(moved, scanned.floor[eye] - limit, scanned.floor[eye] + limit);
  }
}

}  // namespace lf
