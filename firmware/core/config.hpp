#pragma once

#include <array>
#include <cstdint>

#include "types.hpp"

// Every number here is one of: a wiring fact checked on this robot, a quantity
// measured on this robot at home, a safety limit, or a gain tuned on a track.
// Each section says which. Nothing comes from a datasheet.
namespace lf {

inline constexpr std::uint32_t kControlHz = 500;
inline constexpr float kControlDt = 1.0F / static_cast<float>(kControlHz);

// --- Wiring facts -----------------------------------------------------------
// Positive duty runs the wheel backwards on a side marked true (owner, by eye,
// 2026-11-11: the left wheel ran away from the ToF).
inline constexpr std::array<bool, kWheelCount> kInvertWheel = {true, false};
// The encoder counts down while its wheel rolls forward on a side marked true
// (measured 2026-11-11: forward gave left +14190, right -14203).
inline constexpr std::array<bool, kWheelCount> kInvertEncoder = {false, true};
// +1 when the gyro reports counter-clockwise (left turn) as positive.
// UNVERIFIED: the first `turn` on the robot settles it; the run stops itself
// with gyro_disagrees if this is wrong.
inline constexpr float kGyroSign = 1.0F;

// --- Measured on this robot ------------------------------------------------------
// The battery voltage the gains below were measured at; duty is corrected for sag.
inline constexpr float kModelVoltage = 11.4F;
// Count difference between the wheels per radian of body turn: from the gyro rate
// against the wheel speed difference during the first turn.
inline constexpr float kTrackCountsPerRad = 3040.0F;
// Distance per encoder count: slope of ToF distance against encoder counts over a
// 300 mm go (0.0331) and back (0.0295), ToF noise about 10 mm. Cross-check: this
// times kTrackCountsPerRad is 95 mm, the wheel spacing (110 mm outer span minus
// the wheel width).
inline constexpr float kMmPerCount = 0.0313F;

// --- Safety limits ------------------------------------------------------------
inline constexpr float kMaxDuty = 0.9F;
// Forward duty may change by at most this much per tick (ADR-024): up slowly so the
// wheels do not slip from a start, down fast so slowing for a corner is not late.
// Starting points; tune by eye on the track.
inline constexpr float kForwardRisePerTick = 0.004F;  // 0 to 0.7 in 0.35 s
inline constexpr float kForwardFallPerTick = 0.02F;
// 3S LiPo: 3.3 V per cell.
inline constexpr float kMinBatteryV = 9.9F;
// A wheel commanded forward at least this hard counts as trying to move; if neither
// wheel turns faster than kStallCps for kStallTicks, the robot is jammed.
inline constexpr float kStallForwardDuty = 0.35F;
inline constexpr float kStallCps = 300.0F;
inline constexpr std::uint64_t kStallTicks = 200;  // 0.4 s at 500 Hz
// Older than this, the gyro is not trusted to steer.
inline constexpr std::uint64_t kGyroStaleUs = 20000;
// Gyro bias is measured over a block of kBiasBlockUs in which the gyro spread
// stays below kStillRateRad and the wheels move by at most kStillCounts. A block
// that fails (the finger that pressed the button, a bump) is thrown away and the
// next one tried, until kSettleTimeoutUs.
inline constexpr std::uint64_t kBiasBlockUs = 250000;
inline constexpr std::uint64_t kSettleTimeoutUs = 3000000;
inline constexpr float kStillRateRad = 0.03F;  // measured at rest: 0.007 peak-to-peak
inline constexpr std::int32_t kStillCounts = 4;
// Encoder yaw and gyro yaw pointing opposite ways beyond this means a sign is wrong.
inline constexpr float kYawDisagreeRad = 0.3F;
inline constexpr std::uint64_t kButtonDebounceUs = 35000;

// --- Motion estimation --------------------------------------------------------
// Wheel speed is the count change over this many control ticks: one tick of
// quantisation would be +-17% at 3000 counts/s. It is used for odometry and stall
// detection only; steering does not wait on it any more.
inline constexpr std::size_t kSpeedWindowTicks = 10;

// --- Facing a heading (tuned on the bench) ---------------------------------------
// Turning in place back to a heading after the self-scan: proportional on the angle
// left, damped by the turn rate, never weaker than what moves the wheels at all.
inline constexpr float kTurnPPerRad = 1.5F;
inline constexpr float kTurnDDutyPerRate = 0.35F;
inline constexpr float kTurnMinDuty = 0.3F;  // above the wheels' dead band (~0.21-0.32)
inline constexpr float kTurnDutyMax = 0.8F;
inline constexpr float kTurnToleranceRad = 0.03F;
inline constexpr float kTurnSettledRateRadS = 0.3F;
// Failure bound for the whole self-scan; progress is judged by the gyro, not time.
inline constexpr std::uint64_t kCalibrateTimeoutUs = 15000000;

// --- Reading the line --------------------------------------------------------
// Dimensionless rules about each eye's own levels, so they do not depend on the
// floor, the light or which way the sensor reads.
inline constexpr std::size_t kMinUsableEyes = 5;
// An eye can tell line from floor only if the two levels are further apart than
// the floor itself wanders over the scan (the noise, tilt and light that eye sees),
// by this factor (3 refused every outdoor eye in sun whose floor wandered by 200;
// 2 accepts 5 of 8 on that same real scan), and by at least kMinSeparationCounts. This is judged
// per eye from its own scan, so a dark floor that leaves little headroom below the ADC rail still
// works while its contrast is real, and a saturated eye is left out.
inline constexpr float kContrastOverSpread = 2.0F;
inline constexpr float kMinSeparationCounts = 100.0F;
// Histogram resolution and how much of the scan counts as "the rare tail".
inline constexpr std::size_t kScanBinWidth = 16;
inline constexpr float kScanTailFraction = 0.02F;
// Polarity is voted per eye from tails this wide, and the floor's wander is read at
// this quantile, so a few glare spikes in one eye do not decide either.
inline constexpr float kPolarityTailFraction = 0.05F;
inline constexpr float kSpreadTailFraction = 0.10F;
// An eye is on the line past half way from floor to line; a quarter way and
// beyond still shifts the centre.
inline constexpr float kLineOnFraction = 0.5F;
// Dark under this fraction of the usable eyes is not a line any more: a crossing or
// a dark area. ASSUMES the line covers a smaller share of the bar than this; the
// log's dark column shows how many eyes the line really covers (2 to 3 so far).
inline constexpr float kWideFraction = 0.75F;
// Light changes along the course: the floor level may drift by up to this much of
// the eye's own contrast, at this rate, and only while the eye surely sees floor.
inline constexpr float kFloorAdaptLimit = 0.5F;
inline constexpr float kFloorAdaptPerS = 0.5F;
inline constexpr float kFloorSureFraction = 0.1F;

// --- Self-scan: swing over the line until every eye has seen both levels -----------
// Each swing turns at a fixed duty until the gyro passes its angle, then the next
// swing turns back the other way, wider.
inline constexpr std::array<float, 6> kScanSwingsRad = {0.5F, -0.5F, 0.9F, -0.9F, 1.3F, -1.3F};
inline constexpr float kScanTurnDuty = 0.7F;

// --- Bar geometry: assembly facts the virtual line depends on ------------------------
// Eyes ahead of the wheel axle: about 90 mm on base PCB v0.7 (ADR-025). Measure on the
// built robot and replace.
inline constexpr float kBarAheadMm = 90.0F;
// Spacing between neighbouring eyes. UNVERIFIED: not measured yet; the test plant
// used 7 mm. Only the virtual line (brick detour) uses it.
inline constexpr float kEyePitchMm = 7.0F;

// --- Following (tuned on a track) -----------------------------------------------------
inline constexpr float kCruiseDuty = 0.7F;
// Steering: turn duty per eye of line offset, plus a term on how fast the offset is
// changing, so the robot starts turning before the line reaches the edge instead of
// after. Negative offset is left, positive turn is left.
inline constexpr float kSteerDutyPerEye = 0.20F;
inline constexpr float kSteerDutyDamping = 0.02F;  // duty per eye/s of drift
inline constexpr float kRateFilterPerS = 6.0F;     // low-pass on the offset rate
// Speed adapts: full on a straight line, less as the offset grows or moves.
// kMinFollowFraction is the floor while still following.
inline constexpr float kSlowByPosition = 0.12F;  // fraction of duty lost per eye of offset
inline constexpr float kSlowByRate = 0.02F;      // fraction of duty lost per eye/s of drift
inline constexpr float kMinFollowFraction = 0.45F;
// While most of the bar is dark (a crossing, a node, a thick mark) the robot crawls
// and keeps its aim. If the line reappears it was a crossing; if not, it searches.
// The search and the end check crawl at this duty too. CHECK on the robot: the
// measured dead band is about 0.32 on the floor, so this may not move it.
inline constexpr float kNodeCrawlDuty = 0.30F;
// Dark that lasts further than this is a failure, not a line to keep crossing. The
// final track's filled square is 164 mm long (track_chungket.png, 1 mm per pixel).
inline constexpr float kNodeStraightMm = 250.0F;
// Searching for a lost line: crawl (kNodeCrawlDuty) until the axle reaches where the
// bar lost the line, then aim this many eyes off-centre: a turn in place through the
// same steering law. 4 eyes (0.8 duty) spun past the new line before the bar caught it.
inline constexpr float kSearchAimEyes = 2.5F;
// How far one side is searched before the other: a little past a right angle.
inline constexpr float kSearchTurnRad = 1.66F;
// Line gone from mid-bar on a straight: roll on this far; still no line is the finish.
inline constexpr float kConfirmMm = 100.0F;
// Rolling to a lost line's corner, a heading error steers like this many eyes per
// radian. Checking for the end, the robot first faces the old heading (to within
// kTurnToleranceRad, as after the self-scan), then rolls on.
inline constexpr float kHoldEyesPerRad = 4.0F;

// --- Brick on the line: followed around as a virtual line on the right (ADR-026) ------
// The ToF closer than this starts a detour.
inline constexpr float kBrickSeeMm = 300.0F;
// Worst case of the 205x95x95 mm brick (FAQ Q-13): 205 mm across and along the line.
inline constexpr float kBrickSizeMm = 205.0F;
// How far right the virtual line runs: half the brick, half the robot (110 mm outer),
// and margin.
inline constexpr float kDetourSideMm = 0.5F * kBrickSizeMm + 55.0F + 50.0F;
// Clear of the brick's face before reaching it, and of its back before turning in:
// the robot reaches back about kDetourTailMm behind the bar.
inline constexpr float kDetourLeadMm = 40.0F;
inline constexpr float kDetourTailMm = 160.0F;
// Length of each smooth side-step of the virtual line.
inline constexpr float kDetourRampMm = 220.0F;
inline constexpr float kDetourDuty = 0.5F;
// Back past the brick, the real line is taken again only while the bar is this close
// to the old line's axis: a line seen further out is another part of the track (Q-15).
inline constexpr float kRejoinCorridorMm = 60.0F;
// The virtual line runs on straight this far past its end before the line counts as lost.
inline constexpr float kDetourSearchMm = 250.0F;

// ToF older than this is not a reading.
inline constexpr std::uint64_t kTofStaleUs = 100000;

// --- Run 2: run 1's effort, read ahead (ADR-024; all tuned on a track) -----------------
// Run 1 keeps one Stretch per this much line; 4096 of them hold 82 m.
inline constexpr float kStretchMm = 20.0F;
inline constexpr float kStretchCounts = kStretchMm / kMmPerCount;
inline constexpr std::size_t kCourseStretches = 4096;
// A stretch where run 1 steered harder than this fraction of kTurnDutyMax is hard.
inline constexpr float kHardEffort = 0.3F;
// Run 2 on a plain line with nothing hard ahead: this many times run 1's duty
// (kCruiseDuty * 1.25 = 0.875, inside kMaxDuty).
inline constexpr float kFastPace = 1.25F;
// How far run 2 reads ahead, in ticks at the speed it has now: the faster it goes,
// the further it looks, with no braking formula.
inline constexpr float kLookaheadTicks = 150.0F;
// A corner or dark patch in run 2 is the same one as in run 1 only within this distance
// of where the record expects it.
inline constexpr float kLandmarkWindowCounts = 300.0F / kMmPerCount;

// --- Log ------------------------------------------------------------------------
inline constexpr std::uint32_t kLogEveryTicks = 10;  // 50 Hz: 4000 records hold 80 s, a whole lap

}  // namespace lf
