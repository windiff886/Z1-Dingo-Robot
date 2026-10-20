#pragma once
#include "controller.hpp"
#include "transport.hpp"
#include <atomic>
#include <functional>
#include <thread>
namespace z1 {
double monotonicTime();
class Runtime {
public:
  Runtime(ModelConfig, Config, std::unique_ptr<Transport>);
  Runtime(ModelConfig, Config, std::function<std::unique_ptr<Transport>()>);
  ~Runtime();
  Controller &controller() { return controller_; }
  void start();
  void stop();

private:
  Controller controller_;
  Config config_;
  std::unique_ptr<Transport> transport_;
  std::atomic<bool> running_{false};
  std::thread thread_;
};
} // namespace z1
