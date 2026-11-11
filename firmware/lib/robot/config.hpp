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
// Fitted to the first bench routine on the floor, 2026-11-11, battery 11.4 V:
// speed (encoder counts/s) against duty. The wheels-free figures measured earlier
// (gain 11700, tau 0.15 s) were wrong for a robot on the ground: it is about
// twice as strong per duty and about four times slower to respond.
inline constexpr std::array<float, kWheelCount> kMotorGain = {22480.0F, 21009.0F};  // per duty
inline constexpr std::array<float, kWheelCount> kMotorTau = {0.62F, 0.55F};         // s
inline constexpr std::array<float, kWheelCount> kMotorDead = {0.21F, 0.22F};        // duty
inline constexpr float kModelVoltage = 11.4F;
// The speed loop is tuned for a closed-loop time constant of tau divided by this.
inline constexpr float kSpeedLoopSpeedup = 2.0F;
// The speed loop's PI only trims what the model gets wrong (turning in place needs
// about 0.2 duty more than rolling), so it may move the duty by at most this much.
inline constexpr float kTrimLimitDuty = 0.3F;
// Count difference between the wheels per radian of body turn: from the gyro rate
// against the wheel speed difference during the first turn.
inline constexpr float kTrackCountsPerRad = 3040.0F;
// Distance per encoder count. PLACEHOLDER until measured against the ToF.
inline constexpr float kMmPerCount = 0.1F;

// --- Safety limits ------------------------------------------------------------
inline constexpr float kMaxDuty = 0.9F;
// 3S LiPo: 3.3 V per cell.
inline constexpr float kMinBatteryV = 9.9F;
// A wheel told to move faster than kStallTargetCps and turning slower than
// kStallRatio of the order for kStallUs is jammed.
inline constexpr float kStallTargetCps = 500.0F;
inline constexpr float kStallRatio = 0.2F;
inline constexpr std::uint64_t kStallUs = 500000;
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
// quantisation would be +-17% at 3000 counts/s.
inline constexpr std::size_t kSpeedWindowTicks = 10;
// Below this the speed loop does not try to hold a target.
inline constexpr float kMinTargetCps = 40.0F;

// --- Manoeuvres (tuned on the bench) -------------------------------------------
inline constexpr float kGoCps = 3000.0F;
inline constexpr float kGoMinCps = 400.0F;
inline constexpr float kGoAccelCpsPerS = 6000.0F;
inline constexpr float kGoApproachPerS = 6.0F;  // speed = this x counts remaining
inline constexpr float kGoToleranceMm = 3.0F;
inline constexpr float kHeadingHoldPerS = 3.0F;
inline constexpr float kHoldOmegaMaxRadS = 0.5F;
inline constexpr float kTurnGainPerS = 4.0F;
inline constexpr float kTurnOmegaMaxRadS = 2.5F;
// Turning in place has to keep this speed until it arrives: below it the wheels
// stop against friction short of the goal (stuck at 72 of 90 degrees on the floor).
inline constexpr float kTurnOmegaMinRadS = 0.8F;
inline constexpr float kTurnToleranceRad = 0.04F;
// After a move ends the wheels are braked for this long, then released.
inline constexpr std::uint64_t kBrakeHoldUs = 200000;
// Failure bound only. Progress is judged by encoder and gyro, never by time.
inline constexpr std::uint64_t kManeuverTimeoutUs = 10000000;

// --- Line sensors (kept until the step 2 rewrite) --------------------------------
inline constexpr float kLineSplitFraction = 0.5F;
inline constexpr float kLineMinContrast = 0.15F;

// ToF older than this is logged as missing.
inline constexpr std::uint64_t kTofStaleUs = 100000;

// --- Log ------------------------------------------------------------------------
inline constexpr std::uint32_t kLogEveryTicks = 5;  // 100 Hz

}  // namespace lf
