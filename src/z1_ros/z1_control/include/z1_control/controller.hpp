#pragma once
#include "model.hpp"
#include <deque>
#include <mutex>
#include <ruckig/ruckig.hpp>
namespace z1 {
class Controller {
public:
  Controller(ModelConfig, Config);
  bool goal(uint64_t, const Vec &, double speed = 1);
  bool poseGoal(uint64_t, const Pose &, double speed = 1);
  bool stream(uint64_t session, uint64_t sequence, const Vec &, double now);
  bool stream(uint64_t session, uint64_t sequence, const Pose &, double now);
  void operate(Operation, uint64_t id = 0);
  bool gains(const Vec &stiffness, const Vec &damping);
  bool gripper(double position);
  Output step(const Feedback &, double now);
  Snapshot snapshot() const;
  std::vector<Event> events();

private:
  using Input = ruckig::InputParameter<6>;
  using Trajectory = ruckig::Trajectory<6>;
  bool prepare(uint64_t, const Vec *, const Pose *, double);
  Input input(const Vec &, const Vec &, const Vec &) const;
  bool pathSafe(const Trajectory &, Model &, double margin, double budget,
                std::string *failure);
  bool beginStop(const std::string &, double);
  void fault(const std::string &);
  void event(uint64_t, Outcome, const std::string &);
  void startStream(Mode, double);
  ModelConfig model_config_;
  Config config_;
  Model model_, planning_model_;
  mutable std::mutex mutex_;
  std::mutex planning_mutex_;
  Snapshot state_;
  std::deque<Event> events_;
  std::deque<std::pair<Operation, uint64_t>> operations_;
  ruckig::Ruckig<6> generator_;
  Vec reference_{Vec::Zero()}, velocity_{Vec::Zero()},
      acceleration_{Vec::Zero()}, last_torque_{Vec::Zero()};
  Vec stream_q_{Vec::Zero()};
  Pose stream_pose_{Pose::Identity()}, impedance_reference_{Pose::Identity()};
  Pose finite_pose_target_{Pose::Identity()};
  double stream_at_{}, motion_at_{}, duration_{}, settled_at_{}, last_step_{},
      gripper_target_{};
  uint64_t stream_sequence_{}, generation_{};
  uint64_t last_goal_id_{};
  bool target_valid_{}, stop_latched_{}, resume_waiting_{},
      gripper_requested_{};
  Mode resume_mode_{Mode::JointStream};
  Trajectory finite_, stop_;
};
ModelConfig defaultModelConfig(const std::string &urdf);
} // namespace z1
