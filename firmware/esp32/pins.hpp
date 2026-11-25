#pragma once

// Every pin the firmware touches, in one place. Source: hardware/wiring.md
// (single-sided base board v0.7). The board also wires BTN_2 (GPIO12) and IMU INT
// (GPIO13); nothing drives them yet.
namespace lf::esp32::pins {

// The bar's IR pin reaches GPIO18 only with jumper J4 on that position.
inline constexpr int kIrCtrl = 18;
// What that pin does to the bar. NOT KNOWN YET (wiring.md: "điều khiển LED?"): find
// out on the board with the console's `irctl` and `monitor`, then set it. kNotUsed
// leaves GPIO18 an input; the other two make the ADC read each eye with its LED on
// and off and publish the difference, which cancels room light.
enum class IrCtrl { kNotUsed, kLedOnHigh, kLedOnLow };
inline constexpr IrCtrl kIrCtrlWiring = IrCtrl::kNotUsed;

// IR D1..D8, left to right on the bar: GPIO10, 9, 3, 8, 7, 6, 5, 4.
// On ADC1, channel = GPIO - 1.
inline constexpr int kIrAdcChannel[8] = {9, 8, 2, 7, 6, 5, 4, 3};
inline constexpr int kBatteryAdcChannel = 0;  // GPIO1
inline constexpr int kMotorLeftIn1 = 38;
inline constexpr int kMotorLeftIn2 = 39;
inline constexpr int kMotorRightIn1 = 41;
inline constexpr int kMotorRightIn2 = 42;
inline constexpr int kI2cSda = 15;
inline constexpr int kI2cScl = 16;
inline constexpr int kEncoderLeftA = 21;
inline constexpr int kEncoderLeftB = 47;
inline constexpr int kEncoderRightA = 40;
inline constexpr int kEncoderRightB = 2;
inline constexpr int kButton = 11;  // START
inline constexpr int kTofShutdown = 17;
inline constexpr int kStatusLed = 48;  // RGB LED on the devkit

}  // namespace lf::esp32::pins
