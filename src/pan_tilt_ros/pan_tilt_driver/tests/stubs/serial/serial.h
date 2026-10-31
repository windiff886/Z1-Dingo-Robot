#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>

// The lifecycle fixture replaces Modbus I/O; the startup fixture uses this port.
namespace serial {
enum bytesize_t { eightbits };
enum parity_t { parity_none };
enum stopbits_t { stopbits_one };
enum flowcontrol_t { flowcontrol_none };
struct Timeout {
  static Timeout simpleTimeout(uint32_t) { return {}; }
};
class IOException : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};
class Serial {
public:
  void setPort(const std::string &port) { port_ = port; }
  void setBaudrate(uint32_t) {}
  void setBytesize(bytesize_t) {}
  void setParity(parity_t) {}
  void setStopbits(stopbits_t) {}
  void setFlowcontrol(flowcontrol_t) {}
  void setTimeout(Timeout) {}
  void open() {
    if (port_ == "unavailable")
      throw IOException("serial open failed");
  }
  void close() {}
  void flush() {}
  size_t write(const uint8_t *, size_t size) { return size; }
  size_t read(uint8_t *, size_t) { return 0; }
private:
  std::string port_;
};
}
