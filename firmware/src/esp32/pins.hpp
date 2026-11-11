#pragma once

// Every pin the firmware touches, in one place. Source: hardware/wiring.md.
namespace lf::esp32::pins {

inline constexpr int kIrAdcChannel[8] = {0, 1, 3, 4, 5, 6, 7, 8};  // ADC1 channels of GPIO1..9
inline constexpr int kBatteryAdcChannel = 9;                       // GPIO10
inline constexpr int kMotorLeftIn1 = 11;
inline constexpr int kMotorLeftIn2 = 12;
inline constexpr int kMotorRightIn1 = 13;
inline constexpr int kMotorRightIn2 = 14;
inline constexpr int kI2cSda = 15;
inline constexpr int kI2cScl = 16;
inline constexpr int kEncoderLeftA = 17;
inline constexpr int kEncoderLeftB = 18;
inline constexpr int kEncoderRightA = 21;
inline constexpr int kEncoderRightB = 47;
inline constexpr int kButton = 40;
inline constexpr int kTofShutdown = 41;
inline constexpr int kStatusLed = 48;

}  // namespace lf::esp32::pins
