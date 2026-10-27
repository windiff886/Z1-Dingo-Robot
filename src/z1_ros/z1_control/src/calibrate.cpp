#include "z1_control/transport.hpp"
#include <algorithm>
#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <set>
#include <stdexcept>
#include <thread>
#include <yaml-cpp/yaml.h>

namespace {
using Clock = std::chrono::steady_clock;
namespace fs = std::filesystem;
volatile std::sig_atomic_t interrupted = 0;
struct Options {
  unsigned samples{}, interval_ms{}, feedback_timeout_ms{}, reply_timeout_ms{};
  double max_speed{}, max_drift{}, zero_tolerance{};
};
struct Arm {
  std::string side;
  z1::McuConfig config;
  std::unique_ptr<z1::McuTransport> transport;
  std::string result{"NOT_ATTEMPTED"}, detail;
};

unsigned integer(const YAML::Node &node, const char *name, unsigned low,
                 unsigned high) {
  const int value = node[name].as<int>();
  if (value < static_cast<int>(low) || value > static_cast<int>(high))
    throw std::runtime_error(std::string(name) +
                             " is outside its supported range");
  return static_cast<unsigned>(value);
}
double positive(const YAML::Node &node, const char *name) {
  const double value = node[name].as<double>();
  if (!std::isfinite(value) || value <= 0)
    throw std::runtime_error(std::string(name) +
                             " must be finite and positive");
  return value;
}
fs::path default_config() {
  // Installed executables live in bin/ or lib/z1_control/. Devel builds fall
  // back to the source config, without depending on the working directory.
  const auto directory = fs::read_symlink("/proc/self/exe").parent_path();
  for (const auto &prefix :
       {directory.parent_path(), directory.parent_path().parent_path()}) {
    const auto path = prefix / "share/z1_control/config/calibration.yaml";
    if (fs::is_regular_file(path))
      return path;
  }
  return Z1_CALIBRATION_CONFIG;
}
Options load_config(const fs::path &path, std::vector<Arm> &arms) {
  const auto yaml = YAML::LoadFile(path.string());
  Options options;
  options.samples = integer(yaml, "sample_count", 2, 1000);
  options.interval_ms = integer(yaml, "sample_interval_ms", 2, 100);
  options.feedback_timeout_ms =
      integer(yaml, "feedback_timeout_ms", 100, 30000);
  options.reply_timeout_ms = integer(yaml, "reply_timeout_ms", 1, 1000);
  options.max_speed = positive(yaml, "max_joint_speed");
  options.max_drift = positive(yaml, "max_position_drift");
  options.zero_tolerance = positive(yaml, "zero_tolerance");
  if (options.samples * options.interval_ms >= options.feedback_timeout_ms)
    throw std::runtime_error(
        "feedback_timeout_ms must exceed the sampling window");
  fs::path communication = yaml["communication_config"].as<std::string>();
  if (communication.is_relative())
    communication = path.parent_path() / communication;
  const auto connections = YAML::LoadFile(communication.string());
  const auto selected = yaml["arms"];
  if (!selected.IsSequence() || selected.size() == 0 || selected.size() > 2)
    throw std::runtime_error(
        "arms must select [left], [right], or [left, right]");
  std::set<std::string> sides, addresses;
  std::set<unsigned> ports;
  for (const auto &entry : selected) {
    Arm arm;
    arm.side = entry.as<std::string>();
    if ((arm.side != "left" && arm.side != "right") ||
        !sides.insert(arm.side).second)
      throw std::runtime_error("arms must contain unique left/right entries");
    const auto controller = connections[arm.side + "_arm"]["controller"];
    arm.config.ip = controller["mcu_ip"].as<std::string>();
    in_addr address{};
    if (inet_pton(AF_INET, arm.config.ip.c_str(), &address) != 1 ||
        !addresses.insert(arm.config.ip).second)
      throw std::runtime_error(
          "selected arms must have distinct valid IPv4 addresses");
    arm.config.local_port = integer(controller, "local_port", 1, 65535);
    if (!ports.insert(arm.config.local_port).second)
      throw std::runtime_error("selected arms must use distinct local ports");
    arm.config.period = options.interval_ms * 0.001;
    arm.config.receive_budget_us =
        integer(controller, "receive_budget_us", 1, 10000);
    if (arm.config.receive_budget_us >= options.interval_ms * 1000)
      throw std::runtime_error(
          "receive_budget_us must be smaller than sample_interval_ms");
    arms.push_back(std::move(arm));
  }
  return options;
}

z1::Feedback sample(Arm &arm, const Options &options, bool verify_zero) {
  const auto deadline =
      Clock::now() + std::chrono::milliseconds(options.feedback_timeout_ms);
  unsigned count = 0;
  z1::Vec minimum, maximum;
  std::string reason = "no fresh feedback";
  while (Clock::now() < deadline) {
    if (interrupted)
      throw std::runtime_error("interrupted");
    const double now =
        std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
    const auto exchange = arm.transport->exchange(z1::Output{}, now);
    const auto &f = exchange.feedback;
    if (!exchange.sent || f.motor_fault || f.transport_fault)
      throw std::runtime_error(
          "passive feedback has a transport or arm motor fault");
    if (!f.fresh || !f.q.allFinite() || !f.dq.allFinite()) {
      count = 0;
      reason = "no fresh finite six-joint feedback";
    } else if (f.dq.cwiseAbs().maxCoeff() > options.max_speed) {
      count = 0;
      reason = "arm joints are moving";
    } else if (verify_zero &&
               f.q.cwiseAbs().maxCoeff() > options.zero_tolerance) {
      count = 0;
      reason = "arm joint position exceeds zero_tolerance";
    } else {
      if (count == 0)
        minimum = maximum = f.q;
      minimum = minimum.cwiseMin(f.q);
      maximum = maximum.cwiseMax(f.q);
      if ((maximum - minimum).maxCoeff() > options.max_drift) {
        count = 0;
        reason = "arm joint positions are drifting";
      } else if (++count >= options.samples) {
        return f;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(options.interval_ms));
  }
  throw std::runtime_error("feedback check timed out: " + reason);
}

void print_summary(const std::vector<Arm> &arms) {
  std::cout << "\nCalibration results:\n";
  for (const auto &arm : arms)
    std::cout << arm.side << ": " << arm.result
              << (arm.detail.empty() ? "" : " - " + arm.detail) << '\n';
}
} // namespace

int main(int argc, char **argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout
        << "Usage: z1_calibrate [calibration.yaml]\n"
           "Runtime options and arm selection come from YAML.\n"
           "Stop the arm controllers and manually place arms at mechanical "
           "zero.\n"
           "Each selected arm requires a separate console confirmation.\n";
    return 0;
  }
  std::vector<Arm> arms;
  try {
    if (argc > 2)
      throw std::runtime_error("Usage: z1_calibrate [calibration.yaml]");
    const auto config =
        fs::absolute(argc == 2 ? fs::path(argv[1]) : default_config());
    const auto options = load_config(config, arms);
    struct sigaction action {};
    action.sa_handler = [](int) { interrupted = 1; };
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, nullptr);
    sigaction(SIGTERM, &action, nullptr);
    std::cout
        << "Configuration: " << config << '\n'
        << "Stop both arm controllers before calibration (Disabled still uses "
           "UDP).\n"
           "Support each arm and manually place it at its mechanical zero.\n"
           "This program records that pose; keep it still during "
           "calibration.\n";

    // Reserve/connect every selected endpoint before writing any zero
    // reference.
    for (auto &arm : arms) {
      if (interrupted)
        throw std::runtime_error("interrupted");
      std::cout << "Connecting " << arm.side << " MCU " << arm.config.ip
                << ":8880, local port " << arm.config.local_port << std::endl;
      arm.transport = std::make_unique<z1::McuTransport>(arm.config);
    }
    for (auto &arm : arms) {
      if (interrupted)
        break;
      std::cout << "\nPlace " << arm.side << " arm at mechanical zero.\n"
                << "Type '" << arm.side
                << " ZERO' to calibrate this arm, or anything else to stop: "
                << std::flush;
      std::string confirmation;
      if (!std::getline(std::cin, confirmation) ||
          confirmation != arm.side + " ZERO" || interrupted) {
        arm.detail = "confirmation canceled; no calibration request sent";
        break;
      }
      try {
        const auto before = sample(arm, options, false);
        std::cout << "Six-joint feedback before calibration [rad]: "
                  << std::setprecision(5) << before.q.transpose() << '\n';
        if (interrupted)
          throw std::runtime_error("interrupted before calibration request");
        std::cout << "Calibrating " << arm.side
                  << " (official wait approximately 2 seconds)..." << std::endl;
        // Any exception after entering the vendor call has an uncertain
        // outcome.
        arm.result = "UNKNOWN";
        const auto reply =
            arm.transport->calibrate(options.reply_timeout_ms * 1000);
        arm.detail = reply.detail;
        switch (reply.code) {
        case z1::CalibrationCode::Succeeded: {
          arm.result = "ACKNOWLEDGED_UNVERIFIED";
          const auto after = sample(arm, options, true);
          arm.result = "SUCCEEDED";
          arm.detail = "MCU acknowledged; fresh stationary six-joint zero "
                       "feedback verified";
          std::cout << "Six-joint feedback after calibration [rad]: "
                    << after.q.transpose() << '\n';
          break;
        }
        case z1::CalibrationCode::DeviceRejected:
          arm.result = "DEVICE_REJECTED";
          break;
        case z1::CalibrationCode::NotAttempted:
          arm.result = "NOT_ATTEMPTED";
          break;
        case z1::CalibrationCode::Unknown:
          break;
        }
      } catch (const std::exception &error) {
        arm.detail = error.what();
      }
      if (arm.result != "SUCCEEDED")
        break;
    }
    print_summary(arms);
    return !interrupted && std::all_of(arms.begin(), arms.end(),
                                       [](const Arm &arm) {
                                         return arm.result == "SUCCEEDED";
                                       })
               ? 0
               : 1;
  } catch (const std::exception &error) {
    std::cerr << "Calibration stopped: " << error.what() << '\n';
    print_summary(arms);
    return 1;
  }
}
