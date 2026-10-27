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
  std::string ip;
  unsigned local_port{};
  double period{0.002};
  unsigned receive_budget_us{};
};
enum class CalibrationCode { Succeeded, DeviceRejected, NotAttempted, Unknown };
struct CalibrationReply {
  CalibrationCode code{CalibrationCode::NotAttempted};
  std::string detail;
};
class McuTransport final : public Transport {
public:
  explicit McuTransport(const McuConfig &);
  ~McuTransport();
  Exchange exchange(const Output &, double now) override;
  // Maintenance only: blocks for the vendor's approximately two-second wait.
  // Requires recent passive feedback. Never call from the runtime control loop.
  // One attempt per connection; an unknown result must not be retried blindly.
  CalibrationReply calibrate(unsigned reply_timeout_us);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace z1
