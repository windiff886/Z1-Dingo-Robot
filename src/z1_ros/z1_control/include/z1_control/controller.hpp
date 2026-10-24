#pragma once
#include "cartesian_path.hpp"
#include "model.hpp"
#include "reference.hpp"
#include <deque>
#include <functional>
#include <mutex>
#include <ruckig/ruckig.hpp>
namespace z1 {
class Controller {
public:
  Controller(ModelConfig, Config);
  bool goal(uint64_t, const Vec &, double speed = 1);
  bool poseGoal(uint64_t, const Pose &, double speed = 1);
  // Times are offsets from execution start. A first point after t=0 gets an
  // initial resting point at the current reference. retime=false preserves
  // supplied times and derivatives; retime=true permits global time dilation.
  bool trajectory(uint64_t, const std::vector<TrajectoryPoint> &,
                  bool retime = false);
  bool jointWaypoints(uint64_t, const std::vector<Vec> &, double speed = 1);
  bool moveL(uint64_t, const Pose &, double speed = 1);
  bool linearWaypoints(uint64_t, const std::vector<Pose> &,
                       const LinearPathOptions & = {});
  bool stream(uint64_t session, uint64_t sequence, const Vec &, double now);
  bool stream(uint64_t session, uint64_t sequence, const Pose &, double now);
  bool stream(uint64_t session, uint64_t sequence, const JointReference &,
              double received_at);
  bool stream(uint64_t session, uint64_t sequence, const CartesianReference &,
              double received_at);
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
  bool preparePath(uint64_t,
                   const std::function<JointTrajectory(Model &, const Vec &,
                                                       const Config &)> &,
                   Mode, const Pose *endpoint = nullptr);
  Input input(const Vec &, const Vec &, const Vec &) const;
  bool pathSafe(const Trajectory &, Model &, double margin, double budget,
                std::string *failure, const Vec &uncertainty = Vec::Zero());
  bool pathSafe(const JointTrajectory &, Model &, double margin, double budget,
                std::string *failure, const Vec &uncertainty = Vec::Zero());
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
  ReferenceTimeline<JointReference> joint_references_;
  ReferenceTimeline<CartesianReference> pose_references_;
  Pose impedance_reference_{Pose::Identity()};
  Vec impedance_velocity_{Vec::Zero()}, stiffness_, damping_;
  Pose finite_pose_target_{Pose::Identity()};
  double stream_deadline_{}, motion_at_{}, duration_{}, settled_at_{},
      last_step_{}, gripper_target_{};
  uint64_t stream_sequence_{}, generation_{};
  uint64_t last_goal_id_{};
  bool target_valid_{}, stop_latched_{}, resume_waiting_{},
      gripper_requested_{};
  Mode resume_mode_{Mode::JointStream};
  Trajectory finite_, stop_;
  JointTrajectory planned_;
};
ModelConfig defaultModelConfig(const std::string &urdf);
} // namespace z1
