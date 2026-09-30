#pragma once

#include <cstdint>

#include "system/telemetry_packet.hpp"

namespace lf::sim {

class TelemetryReceiver {
public:
  TelemetryReceiver();
  ~TelemetryReceiver();

  // Ngat copy va move de dam bao socket RAII an toan
  TelemetryReceiver(const TelemetryReceiver&) = delete;
  TelemetryReceiver& operator=(const TelemetryReceiver&) = delete;
  TelemetryReceiver(TelemetryReceiver&&) = delete;
  TelemetryReceiver& operator=(TelemetryReceiver&&) = delete;

  bool open(uint16_t port = kDefaultTelemetryPort);
  void close();

  [[nodiscard]] bool is_open() const { return socket_fd_ >= 0; }
  [[nodiscard]] uint16_t get_port() const { return port_; }
  [[nodiscard]] uint32_t get_total_packets() const { return total_packets_; }

  // Doc can bo dem UDP non-blocking va tra ve goi tin hop le moi nhat
  bool poll_latest(TelemetryPacket& out_packet);

private:
  int socket_fd_ = -1;
  uint16_t port_ = kDefaultTelemetryPort;
  uint32_t total_packets_ = 0;
};

}  // namespace lf::sim
