#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace lf {

constexpr uint32_t kTelemetryMagic = 0x4C463236U;  // 'LF26'
constexpr uint16_t kDefaultTelemetryPort = 9876;

#pragma pack(push, 1)
struct TelemetryPacket {
  uint32_t magic = kTelemetryMagic;
  uint32_t timestamp_ms = 0;
  int32_t enc_left_ticks = 0;
  int32_t enc_right_ticks = 0;
  float centroid_pos_mm = 0.0f;
  std::array<float, 8> ir_raw{};
  float tof_front_m = 0.0f;
  float tof_left_m = 0.0f;
  float tof_right_m = 0.0f;
  float left_pwm = 0.0f;
  float right_pwm = 0.0f;
  float battery_voltage_v = 8.4f;
  uint8_t mode = 0;
  uint8_t fsm_state = 0;
  uint8_t line_lost = 0;
  uint8_t reserved = 0;
  uint32_t checksum = 0;

  [[nodiscard]] static uint32_t compute_checksum(const TelemetryPacket& pkt) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    const auto* byte_ptr = reinterpret_cast<const uint8_t*>(&pkt);
    constexpr size_t kPayloadLen = sizeof(TelemetryPacket) - sizeof(uint32_t);
    uint32_t sum = 0x5A5AA5A5U;
    for (size_t i = 0; i < kPayloadLen; ++i) {
      sum = (sum * 31U) ^ byte_ptr[i];
    }
    return sum;
  }

  [[nodiscard]] bool is_valid() const {
    if (magic != kTelemetryMagic) {
      return false;
    }
    return checksum == compute_checksum(*this);
  }

  void finalize() {
    magic = kTelemetryMagic;
    checksum = compute_checksum(*this);
  }
};
#pragma pack(pop)

}  // namespace lf
