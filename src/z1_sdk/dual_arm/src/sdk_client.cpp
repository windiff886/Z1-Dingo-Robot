#include "z1_official/sdk.hpp"
#include <cmath>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace UNITREE_ARM;

namespace {
double number(const std::string& s) {
  std::size_t used = 0;
  const double value = std::stod(s, &used);
  if (used != s.size() || !std::isfinite(value))
    throw std::invalid_argument("expected a finite number: " + s);
  return value;
}
unsigned port(const std::string& s) {
  const double n = number(s);
  if (n < 1 || n > 65535 || n != std::floor(n))
    throw std::invalid_argument("invalid UDP port: " + s);
  return static_cast<unsigned>(n);
}
void help() {
  std::cout << "Official Z1 SDK client (no automatic home or motion)\n"
    "Options: --controller-port N --sdk-port N --has-gripper true|false --load-kg N\n"
    "Append -- COMMAND ... for one call, or omit it for an interactive session.\n"
    "Commands:\n"
    "  state | passive | home | quit\n"
    "  movej r p y x y z speed [gripper]\n"
    "  movel r p y x y z speed [gripper]\n"
    "  movec roll1 pitch1 yaw1 x1 y1 z1 roll2 pitch2 yaw2 x2 y2 z2 speed [gripper]\n"
    "  label_run NAME | label_save NAME | teach NAME | repeat NAME | calibrate\n"
    "Angles: rad; positions: m; MoveJ speed: rad/s; MoveL/C speed: m/s.\n"
    "quit only disconnects the SDK; it is not a stop/hold command.\n";
}
void count(const std::vector<std::string>& args, std::size_t n) {
  if (args.size() != n) throw std::invalid_argument("wrong number of command arguments");
}
void label(const std::string& s) {
  if (s.empty() || s.size() > 9 || s.find_first_not_of(
      "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") != std::string::npos)
    throw std::invalid_argument("label must contain 1..9 ASCII letters, digits, '_' or '-'");
}
Vec6 posture(const std::vector<std::string>& args, std::size_t offset) {
  Vec6 p;
  for (unsigned i = 0; i < 6; ++i) p[i] = number(args.at(offset + i));
  return p;
}
bool command(unitreeArm& arm, const std::vector<std::string>& args) {
  if (args.empty()) return true;
  const auto& op = args.front();
  if (op == "help") { help(); return true; }
  if (op == "quit") { count(args, 1); return false; }
  if (op == "state") {
    count(args, 1);
    // Read only after joining the library's writer thread.
    arm.sendRecvThread->shutdown();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    do {
      arm.sendRecv();
    } while (arm._ctrlComp->recvState.state == ArmFSMState::INVALID &&
             std::chrono::steady_clock::now() < deadline);
    if (arm._ctrlComp->recvState.state == ArmFSMState::INVALID) {
      arm.sendRecvThread->start();
      throw std::runtime_error("controller has not published a valid state");
    }
    std::cout << "fsm=" << static_cast<int>(arm._ctrlComp->recvState.state)
              << " q=" << arm.lowstate->getQ().transpose()
              << " dq=" << arm.lowstate->getQd().transpose()
              << " tau=" << arm.lowstate->getTau().transpose() << '\n';
    arm.sendRecvThread->start();
  } else if (op == "passive") {
    count(args, 1);
    if (!arm.setFsm(ArmFSMState::PASSIVE)) throw std::runtime_error("PASSIVE rejected by controller");
    std::cout << "PASSIVE acknowledged\n";
  } else if (op == "home") {
    count(args, 1);
    arm.backToStart();
    std::cout << "backToStart returned\n";
  } else if (op == "calibrate") {
    count(args, 1);
    arm.calibration();
    std::cout << "calibration returned\n";
  } else if (op == "label_run" || op == "label_save" || op == "teach" || op == "repeat") {
    count(args, 2);
    label(args[1]);
    if (op == "label_run") arm.labelRun(args[1]);
    else if (op == "label_save") arm.labelSave(args[1]);
    else if (op == "teach") arm.teach(args[1]);
    else arm.teachRepeat(args[1]);
    std::cout << op << " returned\n";
  } else if (op == "movej" || op == "movel" || op == "movec") {
    const bool circle = op == "movec";
    const std::size_t speed_index = circle ? 13 : 7;
    if (args.size() != speed_index + 1 && args.size() != speed_index + 2)
      throw std::invalid_argument("expected posture(s), speed, and optional gripper position");
    const Vec6 first = posture(args, 1);
    const Vec6 second = circle ? posture(args, 7) : Vec6::Zero();
    const double speed = number(args[speed_index]);
    if (speed <= 0 || (op == "movej" && speed > M_PI))
      throw std::invalid_argument("speed must be positive; MoveJ maximum is pi rad/s");
    const bool grip = args.size() == speed_index + 2;
    const double g = grip ? number(args.back()) : 0.0;
    if (grip && (g < -M_PI / 2 || g > 0))
      throw std::invalid_argument("gripper position must be in [-pi/2, 0] rad");
    bool ok;
    if (op == "movej") ok = grip ? arm.MoveJ(first, g, speed) : arm.MoveJ(first, speed);
    else if (op == "movel") ok = grip ? arm.MoveL(first, g, speed) : arm.MoveL(first, speed);
    else ok = grip ? arm.MoveC(first, second, g, speed) : arm.MoveC(first, second, speed);
    if (!ok) throw std::runtime_error("official motion call returned false");
    std::cout << op << " returned true (official SDK result)\n";
  } else throw std::invalid_argument("unknown command: " + op);
  return true;
}
}  // namespace

int main(int argc, char** argv) {
  try {
    unsigned controller_port = 8071, sdk_port = 8072;
    bool gripper = true;
    double load = 0.0;
    std::vector<std::string> once;
    for (int i = 1; i < argc; ++i) {
      const std::string opt(argv[i]);
      if (opt == "--help") { help(); return 0; }
      if (opt == "--") {
        while (++i < argc) once.emplace_back(argv[i]);
        if (once.empty()) throw std::invalid_argument("missing command after --");
        break;
      }
      if (i + 1 == argc) throw std::invalid_argument("missing value for " + opt);
      const std::string value(argv[++i]);
      if (opt == "--controller-port") controller_port = port(value);
      else if (opt == "--sdk-port") sdk_port = port(value);
      else if (opt == "--load-kg") {
        load = number(value);
        if (load < 0) throw std::invalid_argument("load must be nonnegative");
      } else if (opt == "--has-gripper") {
        if (value != "true" && value != "false") throw std::invalid_argument("has-gripper must be true or false");
        gripper = value == "true";
      } else throw std::invalid_argument("unknown option: " + opt);
    }
    if (controller_port == sdk_port) throw std::invalid_argument("SDK and controller ports must differ");
    unitreeArm arm(z1_official::components(controller_port, sdk_port, gripper, load));
    std::cout << std::fixed << std::setprecision(6);
    arm.sendRecvThread->start();
    if (!once.empty()) {
      command(arm, once);
    } else {
      help();
      std::string line;
      while (std::cout << "z1> " && std::getline(std::cin, line)) {
        std::istringstream input(line);
        std::vector<std::string> args;
        for (std::string word; input >> word;) args.push_back(word);
        try { if (!command(arm, args)) break; }
        catch (const std::exception& e) { std::cerr << "Command failed: " << e.what() << '\n'; }
      }
    }
    arm.sendRecvThread->shutdown();
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Z1 SDK client: " << e.what() << '\n';
    return 1;
  }
}
