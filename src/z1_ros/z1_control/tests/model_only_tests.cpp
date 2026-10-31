#include "z1_control/controller.hpp"
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <stdexcept>

using namespace z1;
namespace {
void require(bool value, const char *reason) {
  if (!value)
    throw std::runtime_error(reason);
}
struct Models {
  std::filesystem::path directory;
  explicit Models(const std::string &source) {
    char pattern[] = "/tmp/z1-model-only-XXXXXX";
    auto path = mkdtemp(pattern);
    require(path != nullptr, "temporary model directory failed");
    directory = path;
    std::ifstream input(source);
    require(bool(input), "source model missing");
    const std::string xml((std::istreambuf_iterator<char>(input)), {});
    const auto stripped = std::regex_replace(
        xml, std::regex(R"(<collision\b[^>]*>[\s\S]*?</collision>)"), "");
    require(stripped != xml && stripped.find("<collision") == std::string::npos,
            "fixture did not remove all collision objects");
    write("no_collision.urdf", stripped);
    auto unresolved = stripped;
    const auto link = unresolved.find("<link name=\"link00\">");
    require(link != std::string::npos, "base link missing in fixture");
    unresolved.insert(unresolved.find('>', link) + 1,
                      "<collision><geometry><mesh filename=\"" +
                          (directory / "missing.stl").string() +
                          "\"/></geometry></collision>");
    write("unresolved_mesh.urdf", unresolved);
  }
  ~Models() { std::filesystem::remove_all(directory); }
  void write(const char *name, const std::string &xml) {
    std::ofstream output(directory / name);
    output << xml;
    require(bool(output), "temporary model write failed");
  }
};

void compareModel(const ModelConfig &original, const ModelConfig &candidate) {
  Model a(original), b(candidate);
  require(a.lower() == b.lower() && a.upper() == b.upper() &&
              a.effort() == b.effort() && a.velocityLimit() == b.velocityLimit(),
          "geometry changed joint limits");
  for (int i = 0; i < 5; ++i) {
    Vec q;
    q << .03 * i, .8 + .01 * i, -1.1, .3, .01 * i, -.02 * i;
    const Vec dq = Vec::Constant(.02), ddq = Vec::Constant(-.03);
    const Vec tau = a.inverseDynamics(q, dq, ddq);
    require((a.fk(q).matrix() - b.fk(q).matrix()).norm() < 1e-12 &&
                (a.jacobian(q) - b.jacobian(q)).norm() < 1e-12 &&
                (tau - b.inverseDynamics(q, dq, ddq)).norm() < 1e-12 &&
                (a.forwardDynamics(q, dq, tau) -
                 b.forwardDynamics(q, dq, tau)).norm() < 1e-12,
            "geometry changed kinematics or dynamics");
  }
}

void compareControl(const ModelConfig &original, const ModelConfig &candidate) {
  const Config config;
  Controller a(original, config), b(candidate, config);
  Feedback feedback;
  feedback.q << 0, .8, -1.1, .3, 0, 0;
  double now = 1;
  auto tick = [&] {
    feedback.received_at = now += config.period;
    ++feedback.sequence;
    feedback.fresh = true;
    const auto x = a.step(feedback, now), y = b.step(feedback, now);
    const auto state = b.snapshot();
    require(x.enabled == y.enabled && (x.q - y.q).norm() < 1e-12 &&
                (x.dq - y.dq).norm() < 1e-12 &&
                (x.tau - y.tau).norm() < 1e-12 &&
                a.snapshot().mode == state.mode && state.mode != Mode::Fault,
            "geometry changed control output or mode");
    require(std::isnan(state.collision_distance) && state.collision_pair.empty(),
            "legacy distance status must report unavailable");
    if (x.enabled) {
      feedback.q = x.q;
      feedback.dq = x.dq;
      feedback.tau = x.tau;
    }
  };
  tick();
  a.operate(Operation::Enable);
  b.operate(Operation::Enable);
  tick();
  require(b.snapshot().mode == Mode::Holding, "model-only enable failed");
  Vec goal = feedback.q;
  goal[0] += .04;
  require(a.goal(1, goal, .3) && b.goal(1, goal, .3),
          "model-only goal rejected");
  for (int i = 0; i < 3000 && b.snapshot().mode != Mode::Holding; ++i)
    tick();
  require(b.snapshot().mode == Mode::Holding &&
              (feedback.q - goal).norm() < .001,
          "model-only goal did not finish");
  goal[0] += .1;
  require(a.goal(2, goal, .3) && b.goal(2, goal, .3),
          "second model-only goal rejected");
  for (int i = 0; i < 30; ++i)
    tick();
  a.operate(Operation::Stop);
  b.operate(Operation::Stop);
  for (int i = 0; i < 3000 && b.snapshot().mode != Mode::AwaitResume; ++i)
    tick();
  require(b.snapshot().mode == Mode::AwaitResume, "model-only stop failed");
}
} // namespace
int main(int argc, char **argv) {
  try {
    require(argc == 2, "model argument missing");
    Models fixtures(argv[1]);
    const auto original = defaultModelConfig(argv[1]);
    for (const char *name : {"no_collision.urdf", "unresolved_mesh.urdf"}) {
      auto candidate = original;
      candidate.urdf = (fixtures.directory / name).string();
      compareModel(original, candidate);
      compareControl(original, candidate);
    }
    std::cout << "PASS: absent geometry and unresolved collision mesh preserve "
                 "kinematics, dynamics, goals and controlled stops\n";
  } catch (const std::exception &error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
