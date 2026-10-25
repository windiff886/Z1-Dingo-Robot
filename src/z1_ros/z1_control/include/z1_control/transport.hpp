#pragma once
#include "types.hpp"
#include <memory>
namespace z1 {
struct Exchange {
  Feedback feedback;
  bool sent{};
  std::string error;
};
class Transport {
public:
  virtual ~Transport() = default;
  virtual Exchange exchange(const Output &, double now) = 0;
};
struct McuConfig {
  std::string ip{"192.168.123.110"};
  unsigned local_port{8881};
  double period{0.002};
  unsigned receive_budget_us{1000};
  bool hardware_validated{};
};
class McuTransport final : public Transport {
public:
  explicit McuTransport(const McuConfig &);
  ~McuTransport();
  Exchange exchange(const Output &, double now) override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace z1
