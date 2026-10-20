// ABI-bound replacement of socket I/O only. IOUDP retains all vendor
// packet/handshake logic.
#include "UnitreeArmModule/IOPort/UdpPort.h"
#include "udp_observer.hpp"
#include <cerrno>
#include <chrono>
#include <poll.h>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>
namespace z1::udp {
thread_local Observer *active = nullptr;
}
bool UdpPort::isValidIPv4Address(const std::string &ip) {
  in_addr a;
  return inet_pton(AF_INET, ip.c_str(), &a) == 1;
}
UdpPort::UdpPort(std::string name, uint own, std::string ip, uint dest,
                 size_t timeout)
    : IOPort(name, timeout) {
  if (!z1::udp::active || own > 65535 || dest > 65535 ||
      !isValidIPv4Address(ip))
    throw std::runtime_error("invalid observed UDP context/address");
  socket_fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (socket_fd_ < 0)
    throw std::runtime_error("socket creation failed");
  ownAddr_ = {};
  ownAddr_.sin_family = AF_INET;
  ownAddr_.sin_port = htons(own);
  ownAddr_.sin_addr.s_addr = htonl(INADDR_ANY);
  targetAddr_ = {};
  targetAddr_.sin_family = AF_INET;
  targetAddr_.sin_port = htons(dest);
  inet_pton(AF_INET, ip.c_str(), &targetAddr_.sin_addr);
  if (bind(socket_fd_, reinterpret_cast<sockaddr *>(&ownAddr_),
           sizeof(ownAddr_)) < 0) {
    close(socket_fd_);
    throw std::runtime_error("UDP bind failed (port in use or permission)");
  }
  z1::udp::active->descriptors.push_back(socket_fd_);
}
UdpPort::UdpPort(std::string, uint, size_t) : IOPort("unsupported server", 1) {
  throw std::runtime_error("MCU observer supports client sockets only");
}
IOPortStatus UdpPort::send(uint8_t *data, size_t length) {
  if (!z1::udp::active || length > 1500 || length == 0)
    return IOPortStatus::err;
  if (crc32_open_) {
    if (length < 4)
      return IOPortStatus::err;
    add_crc32(data, length);
  }
  bool ok = sendto(socket_fd_, data, length, MSG_DONTWAIT,
                   reinterpret_cast<sockaddr *>(&targetAddr_),
                   sizeof(targetAddr_)) == static_cast<ssize_t>(length);
  z1::udp::active->sent &= ok;
  return ok ? IOPortStatus::ok : IOPortStatus::err;
}
size_t UdpPort::recvLen(uint8_t *data, size_t length) {
  auto *o = z1::udp::active;
  if (!o || length > 1500 || length == 0)
    return 0;
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::microseconds(o->budget_us);
  for (unsigned n = 0; n < 32; ++n) {
    auto remain = deadline - std::chrono::steady_clock::now();
    if (remain <= std::chrono::steady_clock::duration::zero())
      return 0;
    pollfd fd{socket_fd_, POLLIN, 0};
    auto us =
        std::chrono::duration_cast<std::chrono::microseconds>(remain).count();
    timespec timeout{us / 1000000, (us % 1000000) * 1000};
    int ready = ppoll(&fd, 1, &timeout, nullptr);
    if (ready <= 0)
      return 0;
    sockaddr_in source{};
    socklen_t size = sizeof(source);
    ssize_t got = recvfrom(socket_fd_, _recvBuffTemp.data(),
                           _recvBuffTemp.size(), MSG_DONTWAIT | MSG_TRUNC,
                           reinterpret_cast<sockaddr *>(&source), &size);
    if (got <= 0 || got > static_cast<ssize_t>(length) ||
        source.sin_addr.s_addr != targetAddr_.sin_addr.s_addr ||
        source.sin_port != targetAddr_.sin_port)
      continue;
    memcpy(data, _recvBuffTemp.data(), got);
    o->last_length = got;
    o->received = std::chrono::steady_clock::now();
    if (got == 2 && data[0] == 0x20)
      o->motor_count = data[1];
    if (o->motor_count >= 7) {
      size_t joints = o->motor_count == 8 ? 7 : 6;
      if (static_cast<size_t>(got) == joints * 16)
        o->protocol = 1;
      else if (static_cast<size_t>(got) == joints * 22)
        o->protocol = 2;
    }
    return got;
  }
  return 0;
}
IOPortStatus UdpPort::recv(uint8_t *data, size_t length) {
  size_t got = recvLen(data, length);
  if (!got)
    return IOPortStatus::timeout;
  if (got != length)
    return IOPortStatus::msgbad;
  if (crc32_open_ && (length < 4 || !check_crc32(data, length)))
    return IOPortStatus::crcbad;
  isConnect_ = true;
  if (z1::udp::active)
    z1::udp::active->fresh = true;
  if (z1::udp::active && length <= z1::udp::active->state_bytes.size())
    memcpy(z1::udp::active->state_bytes.data(), data, length);
  return IOPortStatus::ok;
}
