#include "telemetry_receiver.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

namespace lf::sim {

TelemetryReceiver::TelemetryReceiver() = default;

TelemetryReceiver::~TelemetryReceiver() {
  close();
}

bool TelemetryReceiver::open(uint16_t port) {
  close();
  port_ = port;

  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    return false;
  }

  // Cho phep tai su dung cong va dia chi
  int opt = 1;
  (void)setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  // Thiet lap socket o che do non-blocking
  const int flags = fcntl(socket_fd_, F_GETFL, 0);
  if (flags >= 0) {
    (void)fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
  }

  sockaddr_in addr{};
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(port_);

  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    close();
    return false;
  }

  return true;
}

void TelemetryReceiver::close() {
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool TelemetryReceiver::poll_latest(TelemetryPacket& out_packet) {
  if (socket_fd_ < 0) {
    return false;
  }

  bool found_new = false;
  TelemetryPacket buffer{};
  sockaddr_in client_addr{};
  socklen_t addr_len = sizeof(client_addr);

  // Vong lap doc can bo dem de lay goi tin moi nhat (drain buffer)
  while (true) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    auto* raw_client_addr = reinterpret_cast<struct sockaddr*>(&client_addr);
    const ssize_t bytes_read =
      recvfrom(socket_fd_, &buffer, sizeof(buffer), 0, raw_client_addr, &addr_len);

    if (bytes_read < 0) {
      break;  // Khong con goi tin nao trong bo dem
    }

    if (static_cast<size_t>(bytes_read) == sizeof(TelemetryPacket) && buffer.is_valid()) {
      out_packet = buffer;
      found_new = true;
      total_packets_++;
    }
  }

  return found_new;
}

}  // namespace lf::sim
