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

// --- Manoeuvres (tuned on the bench) -------------------------------------------
// go and turn are PID on distance and angle, in duty directly. The D term damps
// the speed/rate so the robot brakes as it arrives; the I term builds up against
// the motor's dead band so it still reaches the goal instead of stalling short.
inline constexpr float kGoPPerCount = 0.0008F;
inline constexpr float kGoIPerCountS = 0.0003F;
inline constexpr float kGoIntegralWindowCounts = 2000.0F;  // I acts only near the goal
inline constexpr float kGoDDutyPerCps = 0.00015F;
inline constexpr float kGoDutyMax = 0.7F;
inline constexpr float kGoToleranceCounts = 96.0F;  // ~3 mm
inline constexpr float kGoStopCps = 200.0F;
inline constexpr float kHoldDutyPerRad = 1.0F;
inline constexpr float kHoldIPerRadS = 1.0F;  // holds the heading through the dead band
inline constexpr float kHoldTurnMaxDuty = 0.5F;
inline constexpr float kTurnPPerRad = 1.5F;
inline constexpr float kTurnIPerRadS = 1.5F;
inline constexpr float kTurnIntegralWindowRad = 0.4F;  // I acts only near the goal
inline constexpr float kTurnDDutyPerRate = 0.35F;
inline constexpr float kTurnDutyMax = 0.8F;
inline constexpr float kTurnToleranceRad = 0.03F;
inline constexpr float kTurnSettledRateRadS = 0.3F;
// After a move ends the wheels are braked for this long, then released.
inline constexpr std::uint64_t kBrakeHoldUs = 200000;
// Failure bound only. Progress is judged by encoder and gyro, never by time.
inline constexpr std::uint64_t kManeuverTimeoutUs = 10000000;

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
inline constexpr std::array<float, 6> kScanSwingsRad = {0.5F, -0.5F, 0.9F, -0.9F, 1.3F, -1.3F};
inline constexpr float kScanTurnDuty = 0.7F;

// --- Following (tuned on a track) -----------------------------------------------------
inline constexpr float kCruiseDuty = 0.7F;
// Steering: turn duty per eye of line offset, plus a term on how fast the offset is
// changing, so the robot starts turning before the line reaches the edge instead of
// after. Negative offset is left, positive turn is left.
inline constexpr float kSteerDutyPerEye = 0.20F;
inline constexpr float kSteerDutyDamping = 0.02F;  // duty per eye/s of drift
inline constexpr float kRateFilterPerS = 6.0F;     // low-pass on the offset rate
// Speed adapts: full on a straight line, less as the offset grows or moves, so the
// corner logic has time to react. kMinFollowFraction is the floor while still following.
inline constexpr float kSlowByPosition = 0.12F;  // fraction of duty lost per eye of offset
inline constexpr float kSlowByRate = 0.02F;      // fraction of duty lost per eye/s of drift
inline constexpr float kMinFollowFraction = 0.45F;
// Line lost with the last sighting within this many eyes of the outermost usable
// eye: it left through the side of the bar, which is what a corner looks like.
inline constexpr float kEdgeInsetEyes = 1.0F;
// While most of the bar is dark (a junction or a thick mark) the robot crawls and
// keeps its heading. If the line reappears straight on it was a crossing; if it
// does not, the corner search begins there, not past the junction.
inline constexpr float kNodeCrawlDuty = 0.30F;
// Dark that lasts further than this is a failure, not a line to keep crossing.
inline constexpr float kNodeStraightMm = 120.0F;
// When the line is lost, roll forward this far before turning, so the wheels are at
// the corner while the bar sits over the new line. This is the bar-to-axle distance
// and is measured once on the real robot.
inline constexpr float kCornerAdvanceMm = 45.0F;
// A corner probe turns this far one way, then the other, looking for the exit line.
inline constexpr float kProbeRad = 1.66F;  // ~95 degrees, a little past a right angle
// Line gone from mid-bar with no junction: roll on this far; still no line is the finish.
inline constexpr float kConfirmMm = 100.0F;
// A reading older than this is not trusted to steer.
inline constexpr std::uint64_t kIrStaleUs = 20000;

// --- Obstacle (a brick on the line), bypassed on the right -------------------------------
// Stop this far from the brick when the ToF sees it.
inline constexpr float kBlockApproachMm = 150.0F;
// The robot arrives at the brick at full line speed; it brakes to this wheel speed
// before it turns, so momentum does not sweep it past the corner.
inline constexpr float kBlockBrakeDuty = 0.4F;
inline constexpr float kBlockStopCps = 300.0F;
inline constexpr float kBlockTurnRad = 1.5707963F;  // 90 degrees
inline constexpr float kBlockTurnDuty = 0.6F;
// Worst-case brick across the line (205 mm), half of it plus half the robot plus margin.
inline constexpr float kBlockSideClearMm = 210.0F;
// The forward leg of the detour: the gap where the robot stopped short of the brick,
// plus the brick's length along the line (205 mm), plus the robot's length and margin.
inline constexpr float kBlockLengthClearMm = 600.0F;
// Bounded search back towards the line after passing the brick.
inline constexpr float kBlockSearchMm = 300.0F;

// ToF older than this is logged as missing.
inline constexpr std::uint64_t kTofStaleUs = 100000;

// --- Log ------------------------------------------------------------------------
inline constexpr std::uint32_t kLogEveryTicks = 10;  // 50 Hz: 4000 records hold 80 s, a whole lap

}  // namespace lf
