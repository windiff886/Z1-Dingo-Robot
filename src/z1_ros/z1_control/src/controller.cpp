#include "z1_control/controller.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <stdexcept>
namespace z1 {
namespace {
using Clock = std::chrono::steady_clock;
double seconds(Clock::time_point a) {
  return std::chrono::duration<double>(Clock::now() - a).count();
}
std::array<double, 6> array(const Vec &v) {
  std::array<double, 6> a;
  std::copy(v.data(), v.data() + 6, a.begin());
  return a;
}
Vec vector(const std::array<double, 6> &a) {
  return Eigen::Map<const Vec>(a.data());
}
bool finiteMotion(Mode m) {
  return m == Mode::JointGoal || m == Mode::PoseGoal ||
         m == Mode::JointTrajectory || m == Mode::CartesianPath;
}
bool moving(Mode m) {
  return finiteMotion(m) || m == Mode::JointStream || m == Mode::PoseStream ||
         m == Mode::Impedance;
}
bool streaming(Mode m) {
  return m == Mode::JointStream || m == Mode::PoseStream ||
         m == Mode::Impedance;
}
template <class Sample>
bool geometryPathSafe(Sample sample, double end, const Config &config,
                      Model &model, double margin, double budget,
                      std::string *failure, const Vec &uncertainty,
                      Clock::time_point started) {
  auto fail = [failure](const std::string &reason) {
    if (failure)
      *failure = reason;
    return false;
  };
  double at = 0;
  const double acceleration = model.distanceMotionBound(config.acceleration);
  for (unsigned count = 0; count < config.max_collision_samples; ++count) {
    const auto point = sample(at);
    if (!model.withinLimits(point.position))
      return fail("trajectory sample outside joint limits");
    if (seconds(started) > budget)
      return fail("collision validation time budget exceeded");
    const auto result = model.clearanceSample(point.position, uncertainty);
    if (seconds(started) > budget)
      return fail("collision validation time budget exceeded");
    const double clearance = result.distance - margin;
    if (clearance <= 0)
      return fail("collision margin violated: " + std::string(result.pair));
    if (at >= end)
      return true;
    const double speed = model.distanceMotionBound(point.velocity);
    const double allowance = 0.8 * clearance;
    const double step = std::min(
        0.05,
        2 * allowance /
            (std::sqrt(speed * speed + 2 * acceleration * allowance) + speed));
    if (step < 1e-6)
      return fail("collision validation resolution limit");
    at = std::min(end, at + step);
  }
  return fail("collision validation sample budget exceeded");
}
} // namespace
ModelConfig defaultModelConfig(const std::string &path) {
  ModelConfig c;
  c.urdf = path;
  // Adjacent mounting envelopes overlap by design; these are explicit model
  // rules.
  c.ignored_pairs = {{"link00", "link01"}, {"link01", "link02"},
                     {"link02", "link03"}, {"link03", "link04"},
                     {"link04", "link05"}, {"link05", "link06"},
                     {"link06", "tool"},   {"link00", "link02"},
                     {"link04", "link06"}, {"link05", "tool"}};
  return c;
}
Controller::Controller(ModelConfig m, Config c)
    : model_config_(m), config_(c), model_(m), planning_model_(m),
      generator_(c.period), stiffness_(c.stiffness), damping_(c.damping) {
  if (!std::isfinite(c.period) || c.period < 0.001 || c.period > 0.02 ||
      !std::isfinite(c.feedback_timeout) || c.feedback_timeout < c.period ||
      !std::isfinite(c.stream_timeout) || c.stream_timeout < c.period ||
      !std::isfinite(c.collision_margin) || c.collision_margin <= 0 ||
      !std::isfinite(c.collision_warning) ||
      c.collision_warning <= c.collision_margin ||
      c.max_collision_samples == 0 ||
      c.validation_budget <= 0 || c.lookahead_budget <= 0 ||
      c.lookahead_budget >= c.period)
    throw std::invalid_argument("invalid timing/collision configuration");
  for (double value : {c.reference_preview, c.reference_extrapolation,
                       c.reference_velocity_filter, c.reference_horizon,
                       c.impedance_gain_time_constant})
    if (!std::isfinite(value) || value <= 0)
      throw std::invalid_argument("invalid reference timing");
  if (c.reference_preview > c.reference_extrapolation ||
      c.reference_extrapolation > c.stream_timeout)
    throw std::invalid_argument("reference prediction exceeds input validity");
  for (auto *v : {&c.velocity, &c.acceleration, &c.jerk, &c.torque,
                  &c.torque_rate, &c.kp, &c.kd, &c.stiffness, &c.damping})
    if (!v->allFinite() || (v->array() <= 0).any())
      throw std::invalid_argument("invalid positive control limits/gains");
  config_.torque = config_.torque.cwiseMin(model_.effort());
  if (!std::isfinite(c.validation_budget) ||
      !std::isfinite(c.lookahead_budget) ||
      !std::isfinite(c.position_tolerance) || c.position_tolerance <= 0 ||
      !std::isfinite(c.velocity_tolerance) || c.velocity_tolerance <= 0 ||
      !std::isfinite(c.settle_timeout) || c.settle_timeout <= 0 ||
      !std::isfinite(c.tcp_position_tolerance) ||
      c.tcp_position_tolerance <= 0 ||
      !std::isfinite(c.tcp_orientation_tolerance) ||
      c.tcp_orientation_tolerance <= 0 ||
      (c.velocity.array() > model_.velocityLimit().array()).any() ||
      (c.kp.array() > 5000).any() || (c.kd.array() > 200).any() ||
      (c.stiffness.head<3>().array() > 500).any() ||
      (c.stiffness.tail<3>().array() > 30).any() ||
      (c.damping.array() > 100).any())
    throw std::invalid_argument("control gains/limits exceed supported bounds");
  state_.reason = "startup disabled";
  state_.model_revision = model_.revision();
  std::random_device entropy;
  state_.session = (static_cast<uint64_t>(entropy()) << 32) ^ entropy();
}
Controller::Input Controller::input(const Vec &q, const Vec &v,
                                    const Vec &a) const {
  Input in;
  in.current_position = array(q);
  in.current_velocity = array(v);
  in.current_acceleration = array(a);
  in.max_velocity = array(config_.velocity);
  in.max_acceleration = array(config_.acceleration);
  in.max_jerk = array(config_.jerk);
  return in;
}
void Controller::event(uint64_t id, Outcome out, const std::string &why) {
  if (events_.size() == 128)
    events_.pop_front();
  events_.push_back({id, out, why});
}
bool Controller::pathSafe(const Trajectory &t, Model &model, double margin,
                          double budget, std::string *failure,
                          const Vec &uncertainty) {
  auto fail = [failure](const std::string &reason) {
    if (failure)
      *failure = reason;
    return false;
  };
  auto start = Clock::now();
  double end = t.get_duration();
  if (!std::isfinite(end) || end < 0)
    return fail("invalid trajectory duration");
  // On Ruckig 0.9.2 zero-duration profiles contain unspecified intermediate
  // positions. Only evaluate initialized, positive-duration polynomial
  // segments.
  std::array<double, 6> endpoint, endpoint_v, endpoint_a;
  t.at_time(0, endpoint, endpoint_v, endpoint_a);
  if (!model.withinLimits(vector(endpoint)))
    return fail("trajectory start outside joint limits");
  t.at_time(end, endpoint, endpoint_v, endpoint_a);
  if (!model.withinLimits(vector(endpoint)))
    return fail("trajectory end outside joint limits");
  if (end > 0)
    for (const auto &section : t.get_profiles())
      for (int axis = 0; axis < 6; ++axis) {
        const auto &p = section[axis];
        auto segment = [&](double h, double pos, double vel, double accel,
                           double jerk) {
          if (h <= 0)
            return true;
          if (!std::isfinite(h) || !std::isfinite(pos) || !std::isfinite(vel) ||
              !std::isfinite(accel) || !std::isfinite(jerk))
            return false;
          auto point = [&](double x) {
            double q =
                pos + vel * x + 0.5 * accel * x * x + jerk * x * x * x / 6;
            return std::isfinite(q) && q >= model.lower()[axis] - 1e-10 &&
                   q <= model.upper()[axis] + 1e-10;
          };
          if (!point(0) || !point(h))
            return false;
          if (std::abs(jerk) < 1e-14) {
            if (std::abs(accel) > 1e-14) {
              double root = -vel / accel;
              if (root > 0 && root < h && !point(root))
                return false;
            }
          } else {
            double discriminant = accel * accel - 2 * jerk * vel;
            if (discriminant >= 0) {
              for (double root : {(-accel + std::sqrt(discriminant)) / jerk,
                                  (-accel - std::sqrt(discriminant)) / jerk})
                if (root > 0 && root < h && !point(root))
                  return false;
            }
          }
          return true;
        };
        if (p.brake.duration > 0)
          for (int i = 0; i < 2; ++i)
            if (p.brake.t[i] > 0 &&
                !segment(p.brake.t[i], p.brake.p[i], p.brake.v[i], p.brake.a[i],
                         p.brake.j[i]))
              return fail("braking polynomial outside joint limits");
        for (int i = 0; i < 7; ++i)
          if (p.t[i] > 0 && !segment(p.t[i], p.p[i], p.v[i], p.a[i], p.j[i]))
            return fail("trajectory polynomial outside joint limits");
      }
  return geometryPathSafe(
      [&t](double at) {
        std::array<double, 6> q, v, a;
        t.at_time(at, q, v, a);
        TrajectorySample point;
        point.position = vector(q);
        point.velocity = vector(v);
        point.acceleration = vector(a);
        return point;
      },
      end, config_, model, margin, budget, failure, uncertainty, start);
}
bool Controller::pathSafe(const JointTrajectory &t, Model &model, double margin,
                          double budget, std::string *failure,
                          const Vec &uncertainty) {
  return geometryPathSafe([&t](double at) { return t.sample(at); },
                          t.duration(), config_, model, margin, budget, failure,
                          uncertainty, Clock::now());
}

bool Controller::prepare(uint64_t id, const Vec *q, const Pose *pose,
                         double speed) {
  Vec seed;
  uint64_t generation;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.mode != Mode::Holding || id == 0 || id <= last_goal_id_ ||
        !std::isfinite(speed) || speed <= 0 || speed > 1) {
      event(id, Outcome::Rejected,
            "requires holding, increasing positive goal ID and speed in (0,1]");
      return false;
    }
    seed = reference_;
    generation = generation_;
  }
  std::lock_guard<std::mutex> planner_lock(planning_mutex_);
  Vec target;
  bool okay = true;
  std::string reason = "accepted";
  Trajectory candidate;
  if (pose)
    okay = planning_model_.ik(*pose, seed, target);
  else
    target = *q;
  if (!okay)
    reason = "IK failed";
  else if (!planning_model_.withinLimits(target)) {
    okay = false;
    reason = "joint target outside limits";
  }
  if (okay) {
    auto in = input(seed, Vec::Zero(), Vec::Zero());
    in.target_position = array(target);
    for (auto &v : in.max_velocity)
      v *= speed;
    try {
      ruckig::Ruckig<6> planner(config_.period);
      okay = planner.calculate(in, candidate) == ruckig::Result::Working;
      if (okay)
        okay = pathSafe(candidate, planning_model_,
                        config_.collision_margin + 0.01,
                        config_.validation_budget, &reason);
      else
        reason = "finite trajectory generator error";
    } catch (const std::exception &e) {
      okay = false;
      reason = e.what();
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!okay) {
    event(id, Outcome::Rejected, reason);
    return false;
  }
  if (generation != generation_ || state_.mode != Mode::Holding ||
      id <= last_goal_id_ || (reference_ - seed).norm() > 1e-6) {
    event(id, Outcome::Rejected, "state changed during validation");
    return false;
  }
  last_goal_id_ = id;
  finite_ = candidate;
  duration_ = candidate.get_duration();
  state_.trajectory_duration = duration_;
  state_.trajectory_time_scale = 1;
  motion_at_ = last_step_;
  settled_at_ = 0;
  if (pose)
    finite_pose_target_ = *pose;
  state_.active_id = id;
  state_.mode = pose ? Mode::PoseGoal : Mode::JointGoal;
  state_.reason = "executing";
  ++generation_;
  event(id, Outcome::Accepted, "trajectory validated");
  return true;
}
bool Controller::goal(uint64_t id, const Vec &q, double s) {
  return prepare(id, &q, nullptr, s);
}
bool Controller::poseGoal(uint64_t id, const Pose &p, double s) {
  return prepare(id, nullptr, &p, s);
}
bool Controller::preparePath(
    uint64_t id,
    const std::function<JointTrajectory(Model &, const Vec &, const Config &)>
        &planner,
    Mode mode, const Pose *endpoint) {
  Vec seed;
  Config limits;
  uint64_t generation;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.mode != Mode::Holding || id == 0 || id <= last_goal_id_) {
      event(id, Outcome::Rejected,
            "requires holding and increasing positive goal ID");
      return false;
    }
    seed = reference_;
    generation = generation_;
    limits = config_;
  }
  // Polynomial construction, IK, retiming and whole-path geometry checks run
  // on the caller's thread, with a separate model and no control mutex held.
  std::lock_guard<std::mutex> planner_lock(planning_mutex_);
  JointTrajectory candidate;
  std::string reason;
  bool okay = false;
  try {
    candidate = planner(planning_model_, seed, limits);
    if (candidate.empty())
      throw std::invalid_argument("empty trajectory");
    const auto first = candidate.sample(0);
    const auto last = candidate.sample(candidate.duration());
    if ((first.position - seed).cwiseAbs().maxCoeff() > 1e-6 ||
        first.velocity.cwiseAbs().maxCoeff() > 1e-8 ||
        first.acceleration.cwiseAbs().maxCoeff() > 1e-8)
      throw std::invalid_argument(
          "trajectory must start at the resting reference");
    if (last.velocity.cwiseAbs().maxCoeff() > 1e-8 ||
        last.acceleration.cwiseAbs().maxCoeff() > 1e-8)
      throw std::invalid_argument("finite trajectory must end at rest");
    okay = pathSafe(candidate, planning_model_, limits.collision_margin + 0.01,
                    limits.validation_budget, &reason);
  } catch (const std::exception &error) {
    reason = error.what();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!okay) {
    event(id, Outcome::Rejected, reason);
    return false;
  }
  if (generation != generation_ || state_.mode != Mode::Holding ||
      id <= last_goal_id_ || (reference_ - seed).norm() > 1e-6) {
    event(id, Outcome::Rejected, "state changed during trajectory preparation");
    return false;
  }
  std::swap(planned_, candidate);
  duration_ = planned_.duration();
  motion_at_ = last_step_;
  settled_at_ = 0;
  if (endpoint)
    finite_pose_target_ = *endpoint;
  last_goal_id_ = id;
  state_.trajectory_duration = duration_;
  state_.trajectory_time_scale = planned_.timeScale();
  state_.active_id = id;
  state_.mode = mode;
  state_.reason = "executing prepared trajectory";
  ++generation_;
  event(id, Outcome::Accepted, "trajectory prepared and validated");
  return true;
}
bool Controller::trajectory(uint64_t id,
                            const std::vector<TrajectoryPoint> &points,
                            bool retime) {
  return preparePath(
      id,
      [&](Model &model, const Vec &seed, const Config &config) {
        auto prepared = points;
        if (!prepared.empty() && prepared.front().time_from_start > 0) {
          TrajectoryPoint initial;
          initial.position = seed;
          initial.has_velocity = initial.has_acceleration = true;
          prepared.insert(prepared.begin(), initial);
        }
        return JointTrajectory::timed(prepared, config, model.lower(),
                                      model.upper(), retime);
      },
      Mode::JointTrajectory);
}
bool Controller::jointWaypoints(uint64_t id, const std::vector<Vec> &targets,
                                double speed) {
  return preparePath(
      id,
      [&](Model &model, const Vec &seed, const Config &config) {
        if (targets.empty())
          throw std::invalid_argument("joint waypoints are empty");
        std::vector<Vec> points;
        points.reserve(targets.size() + 1);
        points.push_back(seed);
        points.insert(points.end(), targets.begin(), targets.end());
        return JointTrajectory::waypoints(points, config, model.lower(),
                                          model.upper(), speed);
      },
      Mode::JointTrajectory);
}
bool Controller::moveL(uint64_t id, const Pose &target, double speed) {
  LinearPathOptions options;
  options.speed = speed;
  return linearWaypoints(id, {target}, options);
}
bool Controller::linearWaypoints(uint64_t id, const std::vector<Pose> &targets,
                                 const LinearPathOptions &options) {
  return preparePath(
      id,
      [&](Model &model, const Vec &seed, const Config &config) {
        return planLinearPath(model, seed, targets, config, options);
      },
      Mode::CartesianPath, targets.empty() ? nullptr : &targets.back());
}
bool Controller::stream(uint64_t session, uint64_t seq, const Vec &q,
                        double now) {
  JointReference target;
  target.time = now;
  target.position = q;
  return stream(session, seq, target, now);
}
bool Controller::stream(uint64_t session, uint64_t seq, const Pose &p,
                        double now) {
  CartesianReference target;
  target.time = now;
  target.pose = p;
  return stream(session, seq, target, now);
}
bool Controller::stream(uint64_t session, uint64_t seq,
                        const JointReference &target, double received_at) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_.mode != Mode::JointStream || session != state_.session ||
      seq <= stream_sequence_ || !model_.withinLimits(target.position) ||
      !std::isfinite(received_at) ||
      received_at <= last_step_ - config_.stream_timeout ||
      received_at > last_step_ + config_.feedback_timeout ||
      !std::isfinite(target.time) || target.time < received_at ||
      target.time > last_step_ + config_.reference_horizon ||
      (target.has_velocity &&
       (!target.velocity.allFinite() ||
        (target.velocity.cwiseAbs().array() > config_.velocity.array())
            .any())) ||
      (target.has_acceleration &&
       (!target.has_velocity || !target.acceleration.allFinite() ||
        (target.acceleration.cwiseAbs().array() > config_.acceleration.array())
            .any())))
    return false;
  if (!joint_references_.push(target, config_.reference_velocity_filter))
    return false;
  stream_deadline_ =
      std::max(received_at, target.time) + config_.stream_timeout;
  stream_sequence_ = seq;
  target_valid_ = true;
  resume_waiting_ = false;
  return true;
}
bool Controller::stream(uint64_t session, uint64_t seq,
                        const CartesianReference &target, double received_at) {
  std::lock_guard<std::mutex> lock(mutex_);
  if ((state_.mode != Mode::PoseStream && state_.mode != Mode::Impedance) ||
      session != state_.session || seq <= stream_sequence_ ||
      !validPose(target.pose) || !std::isfinite(received_at) ||
      received_at <= last_step_ - config_.stream_timeout ||
      received_at > last_step_ + config_.feedback_timeout ||
      !std::isfinite(target.time) || target.time < received_at ||
      target.time > last_step_ + config_.reference_horizon ||
      (target.has_velocity && !target.velocity.allFinite()) ||
      (target.has_acceleration &&
       (!target.has_velocity || !target.acceleration.allFinite())))
    return false;
  if (!pose_references_.push(target, config_.reference_velocity_filter))
    return false;
  stream_deadline_ =
      std::max(received_at, target.time) + config_.stream_timeout;
  stream_sequence_ = seq;
  target_valid_ = true;
  resume_waiting_ = false;
  return true;
}
void Controller::operate(Operation op, uint64_t id) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (op == Operation::Disable) {
    operations_.clear();
    operations_.emplace_front(op, id);
  } else if (op == Operation::Stop) {
    operations_.erase(std::remove_if(operations_.begin(), operations_.end(),
                                     [](const auto &entry) {
                                       return entry.first != Operation::Disable;
                                     }),
                      operations_.end());
    operations_.emplace_back(op, id);
  } else if (operations_.size() < 16)
    operations_.emplace_back(op, id);
  else
    event(id, Outcome::Rejected, "control mailbox full");
}
bool Controller::gains(const Vec &k, const Vec &d) {
  std::lock_guard<std::mutex> lock(mutex_);
  if ((state_.mode != Mode::Holding && state_.mode != Mode::Impedance) ||
      !k.allFinite() || !d.allFinite() || (k.array() <= 0).any() ||
      (d.array() <= 0).any() || (k.head<3>().array() > 500).any() ||
      (k.tail<3>().array() > 30).any() || (d.array() > 100).any())
    return false;
  config_.stiffness = k;
  config_.damping = d;
  return true;
}
bool Controller::gripper(double q) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!state_.feedback.has_gripper || state_.mode != Mode::Holding ||
      !std::isfinite(q) || q < -1 || q > 0)
    return false;
  gripper_target_ = q;
  gripper_requested_ = true;
  return true;
}
void Controller::fault(const std::string &why) {
  if (state_.active_id)
    event(state_.active_id, Outcome::Failed, why);
  state_.mode = Mode::Fault;
  state_.reason = why;
  state_.active_id = 0;
  target_valid_ = false;
  gripper_requested_ = false;
  last_torque_.setZero();
  ++state_.session;
  ++generation_;
}
void Controller::startStream(Mode m, double now) {
  state_.mode = m;
  resume_mode_ = m;
  target_valid_ = false;
  resume_waiting_ = true;
  stream_deadline_ = now + config_.stream_timeout;
  stream_sequence_ = 0;
  ++state_.session;
  ++generation_;
  state_.reason = "waiting for new session target";
  state_.trajectory_duration = 0;
  state_.trajectory_time_scale = 1;
  impedance_reference_ = model_.fk(state_.feedback.q);
  impedance_velocity_.setZero();
  JointReference joints;
  joints.time = now;
  joints.position = reference_;
  joints.velocity = velocity_;
  joints.acceleration = acceleration_;
  joints.has_velocity = joints.has_acceleration = true;
  joint_references_.reset(joints);
  CartesianReference pose;
  pose.time = now;
  pose.pose = impedance_reference_;
  pose.has_velocity = pose.has_acceleration = true;
  pose_references_.reset(pose);
}
bool Controller::beginStop(const std::string &why, double now) {
  resume_mode_ = streaming(state_.mode) ? state_.mode : Mode::Holding;
  if (state_.mode == Mode::Impedance || state_.mode == Mode::Holding) {
    reference_ = state_.feedback.q;
    velocity_ = state_.feedback.dq;
    acceleration_.setZero();
    impedance_reference_ = model_.fk(state_.feedback.q);
  }
  auto in = input(reference_, velocity_, acceleration_);
  in.control_interface = ruckig::ControlInterface::Velocity;
  in.target_velocity.fill(0);
  in.target_acceleration.fill(0);
  try {
    if (generator_.calculate(in, stop_) != ruckig::Result::Working) {
      fault("stop generator failed");
      return false;
    }
  } catch (const std::exception &) {
    fault("stop generator exception");
    return false;
  }
  std::string failure;
  if (!pathSafe(stop_, model_, config_.collision_margin,
                config_.lookahead_budget, &failure,
                (state_.feedback.q - reference_).cwiseAbs())) {
    fault("stop trajectory invalid: " + failure);
    return false;
  }
  if (state_.active_id)
    event(state_.active_id, Outcome::Stopped, why);
  state_.active_id = 0;
  state_.mode = Mode::Stopping;
  state_.reason = why;
  stop_latched_ = true;
  target_valid_ = false;
  gripper_requested_ = false;
  motion_at_ = now;
  duration_ = stop_.get_duration();
  ++state_.session;
  ++generation_;
  return true;
}
Output Controller::step(const Feedback &feedback, double now) {
  auto began = Clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  Output out;
  state_.feedback_valid = false;
  // DISABLE is applied even when measurements or the control clock are invalid.
  if (!operations_.empty() && operations_.front().first == Operation::Disable) {
    auto id = operations_.front().second;
    operations_.pop_front();
    if (state_.active_id)
      event(state_.active_id, Outcome::Canceled, "disabled");
    state_.mode = Mode::Disabled;
    state_.reason = "disabled";
    state_.active_id = 0;
    target_valid_ = false;
    gripper_requested_ = false;
    last_torque_.setZero();
    ++state_.session;
    ++generation_;
    event(id, Outcome::Accepted, "disabled; passive output");
  }
  auto reject_operations = [this] {
    while (!operations_.empty()) {
      event(operations_.front().second, Outcome::Rejected,
            "valid feedback and timing required");
      operations_.pop_front();
    }
  };
  double dt = last_step_ == 0 ? config_.period : now - last_step_;
  last_step_ = now;
  try {
    if (!std::isfinite(now) || dt <= 0 || dt > config_.feedback_timeout) {
      if (state_.mode != Mode::Disabled)
        fault("control clock/deadline failure");
      reject_operations();
      return out;
    }
    state_.loop_period = dt;
    state_.max_loop_period = std::max(state_.max_loop_period, dt);
    bool new_feedback =
        feedback.fresh && feedback.sequence > state_.feedback.sequence;
    if (new_feedback)
      state_.feedback = feedback;
    state_.feedback.fresh = new_feedback;
    if (feedback.transport_fault && state_.mode != Mode::Disabled) {
      fault("transport send/backlog fault");
      reject_operations();
      return out;
    }
    auto &f = state_.feedback;
    bool valid =
        f.sequence > 0 && std::isfinite(f.received_at) &&
        f.received_at <= now &&
        now - f.received_at <= config_.feedback_timeout && f.q.allFinite() &&
        f.dq.allFinite() && f.tau.allFinite() &&
        model_.withinLimits(f.q, -0.02) &&
        (f.dq.cwiseAbs().array() <= 1.25 * config_.velocity.array()).all() &&
        !f.motor_fault && !feedback.transport_fault &&
        (!f.has_gripper ||
         (std::isfinite(f.gripper_q) && std::isfinite(f.gripper_dq) &&
          std::isfinite(f.gripper_tau) && f.gripper_q >= -1.02 &&
          f.gripper_q <= 0.02));
    state_.feedback_valid = valid;
    if (!valid) {
      if (state_.mode != Mode::Disabled && state_.mode != Mode::Fault)
        fault("feedback stale, invalid or device fault");
      reject_operations();
      return out;
    }
    state_.tcp = model_.fk(f.q);
    auto clearance = model_.clearanceSample(f.q);
    state_.collision_distance = clearance.distance;
    state_.collision_pair = clearance.pair;
    while (!operations_.empty()) {
      auto [op, id] = operations_.front();
      operations_.pop_front();
      if (op == Operation::Disable) {
        if (state_.active_id)
          event(state_.active_id, Outcome::Canceled, "disabled");
        state_.mode = Mode::Disabled;
        state_.active_id = 0;
        target_valid_ = false;
        gripper_requested_ = false;
        ++state_.session;
        ++generation_;
        last_torque_.setZero();
      } else if (op == Operation::Enable && state_.mode == Mode::Disabled &&
                 clearance.distance > config_.collision_warning &&
                 f.dq.cwiseAbs().maxCoeff() < config_.velocity_tolerance) {
        reference_ = f.q;
        velocity_.setZero();
        acceleration_.setZero();
        gripper_target_ = f.gripper_q;
        state_.mode = Mode::Holding;
        state_.reason = "enabled";
        ++generation_;
      } else if (op == Operation::Stop &&
                 (moving(state_.mode) || state_.mode == Mode::Holding))
        beginStop("user stop", now);
      else if (op == Operation::Resume && state_.mode == Mode::AwaitResume &&
               clearance.distance > config_.collision_warning &&
               f.dq.cwiseAbs().maxCoeff() < config_.velocity_tolerance) {
        reference_ = f.q;
        velocity_.setZero();
        acceleration_.setZero();
        if (resume_mode_ == Mode::Holding) {
          state_.mode = Mode::Holding;
          state_.reason = "stopped motion acknowledged";
          ++generation_;
        } else
          startStream(resume_mode_, now);
      } else if (op == Operation::Resume && state_.mode == Mode::Fault &&
                 clearance.distance > config_.collision_warning &&
                 f.dq.cwiseAbs().maxCoeff() < config_.velocity_tolerance) {
        state_.mode = Mode::Disabled;
        state_.reason = "fault acknowledged; enable required";
        ++generation_;
      } else if (op == Operation::Hold && state_.mode == Mode::AwaitResume &&
                 clearance.distance > config_.collision_warning &&
                 f.dq.cwiseAbs().maxCoeff() < config_.velocity_tolerance) {
        reference_ = f.q;
        velocity_.setZero();
        acceleration_.setZero();
        state_.mode = Mode::Holding;
        state_.reason = "explicit hold; select a new mode";
        target_valid_ = false;
        ++state_.session;
        ++generation_;
      } else if (state_.mode == Mode::Holding &&
                 (op == Operation::JointStream || op == Operation::PoseStream ||
                  op == Operation::Impedance)) {
        startStream(op == Operation::JointStream  ? Mode::JointStream
                    : op == Operation::PoseStream ? Mode::PoseStream
                                                  : Mode::Impedance,
                    now);
      } else {
        event(id, Outcome::Rejected, "operation not allowed in current state");
        continue;
      }
      event(id, Outcome::Accepted, "control operation applied");
    }
    if (state_.mode == Mode::Disabled || state_.mode == Mode::Fault)
      return out;
    if (clearance.distance <= config_.collision_margin) {
      fault("measured geometric collision risk");
      return out;
    }
    if (streaming(state_.mode) && now > stream_deadline_)
      beginStop("stream input timeout", now);
    if ((moving(state_.mode) || state_.mode == Mode::Holding) &&
        clearance.distance < config_.collision_warning)
      beginStop("collision warning distance", now);
    if (state_.mode == Mode::Fault)
      return out;
    Vec next_q = reference_, next_v = velocity_, next_a = acceleration_;
    if (state_.mode == Mode::JointTrajectory ||
        state_.mode == Mode::CartesianPath) {
      const auto point = planned_.sample(std::max(0.0, now - motion_at_));
      next_q = point.position;
      next_v = point.velocity;
      next_a = point.acceleration;
    } else if (state_.mode == Mode::JointGoal ||
               state_.mode == Mode::PoseGoal || state_.mode == Mode::Stopping) {
      std::array<double, 6> q, v, a;
      (state_.mode == Mode::Stopping ? stop_ : finite_)
          .at_time(std::max(0.0, now - motion_at_), q, v, a);
      next_q = vector(q);
      next_v = vector(v);
      next_a = vector(a);
    } else if (target_valid_ && streaming(state_.mode)) {
      Input in = input(reference_, velocity_, acceleration_);
      if (state_.mode == Mode::JointStream) {
        joint_references_.prune(now);
        const auto target = joint_references_.sample(
            now + config_.reference_preview, config_.reference_extrapolation);
        in.target_position = array(
            target.position.cwiseMax(model_.lower()).cwiseMin(model_.upper()));
        in.target_velocity = array(target.velocity.cwiseMax(-config_.velocity)
                                       .cwiseMin(config_.velocity));
        in.target_acceleration =
            array(target.acceleration.cwiseMax(-config_.acceleration)
                      .cwiseMin(config_.acceleration));
        in.minimum_duration = config_.reference_preview;
      } else {
        pose_references_.prune(now);
        const auto target = pose_references_.sample(
            now + config_.reference_preview, config_.reference_extrapolation);
        Vec v = model_
                    .differentialIk(reference_, target.pose,
                                    state_.mode == Mode::Impedance ? 0.5 : 1.0,
                                    target.velocity)
                    .cwiseMax(-config_.velocity)
                    .cwiseMin(config_.velocity);
        in.control_interface = ruckig::ControlInterface::Velocity;
        in.target_velocity = array(v);
        in.target_acceleration.fill(0);
        if (state_.mode == Mode::Impedance) {
          const auto equilibrium = pose_references_.sample(
              now + config_.period, config_.reference_extrapolation);
          impedance_reference_ = equilibrium.pose;
          impedance_velocity_ = equilibrium.velocity;
        }
      }
      // Bound requested terminal velocity by the remaining joint travel. This
      // applies to dynamic joint references as well as differential IK.
      for (int i = 0; i < 6; ++i) {
        const double v = in.target_velocity[i];
        const double distance = v >= 0 ? model_.upper()[i] - reference_[i]
                                       : reference_[i] - model_.lower()[i];
        const double offset =
            config_.acceleration[i] * config_.acceleration[i] / config_.jerk[i];
        const double safe =
            std::max(0.0, std::sqrt(offset * offset +
                                    2 * config_.acceleration[i] *
                                        std::max(0.0, distance - 0.02)) -
                              offset);
        in.target_velocity[i] = std::copysign(std::min(std::abs(v), safe), v);
        const double viable_acc = std::sqrt(
            std::max(0.0, config_.jerk[i] * (config_.velocity[i] -
                                             std::abs(in.target_velocity[i]))));
        in.target_acceleration[i] =
            std::clamp(in.target_acceleration[i], -viable_acc, viable_acc);
      }
      Trajectory trajectory;
      if (generator_.calculate(in, trajectory) != ruckig::Result::Working) {
        fault("stream reference generation failed");
        return out;
      }
      std::array<double, 6> q, v, a;
      // Advance by elapsed time, as finite trajectories do. A delayed wakeup
      // must not permanently slow the reference clock relative to timestamps.
      // Suppress nanosecond-scale subtraction roundoff: tiny residual segments
      // otherwise keep a settled Ruckig reference and geometry cache moving.
      const double elapsed =
          std::abs(dt - config_.period) < 1e-9 ? config_.period : dt;
      trajectory.at_time(elapsed, q, v, a);
      next_q = vector(q);
      next_v = vector(v);
      next_a = vector(a);
    }
    if (moving(state_.mode)) {
      // Every accepted step must preserve a clear, bounded braking path.
      auto brake_in = input(next_q, next_v, next_a);
      brake_in.control_interface = ruckig::ControlInterface::Velocity;
      brake_in.target_velocity.fill(0);
      brake_in.target_acceleration.fill(0);
      Trajectory brake;
      const Vec uncertainty = (f.q - reference_).cwiseAbs();
      std::string failure;
      double horizon = std::max(dt, config_.period);
      const Vec swept = velocity_.cwiseAbs() * horizon +
                        0.5 * config_.acceleration * horizon * horizon;
      auto start_clearance =
          model_.clearanceSample(reference_, uncertainty + swept);
      bool safe =
          start_clearance.distance > config_.collision_margin &&
          generator_.calculate(brake_in, brake) == ruckig::Result::Working &&
          pathSafe(brake, model_, config_.collision_margin,
                   config_.lookahead_budget, &failure, uncertainty);
      if (start_clearance.distance <= config_.collision_margin)
        failure =
            "next interval clearance: " + std::string(start_clearance.pair);
      if (!safe) {
        if (failure.empty())
          failure = "brake generator error";
        if (!beginStop("cannot preserve braking path: " + failure, now))
          return out;
        next_q = reference_;
        next_v = velocity_;
        next_a = acceleration_;
      }
    }
    if (!model_.withinLimits(next_q)) {
      fault("generated joint limit violation");
      return out;
    }
    reference_ = next_q;
    velocity_ = next_v;
    acceleration_ = next_a;
    if (state_.mode == Mode::Stopping && now - motion_at_ >= duration_ &&
        f.dq.cwiseAbs().maxCoeff() < config_.velocity_tolerance) {
      state_.mode = Mode::AwaitResume;
      velocity_.setZero();
      acceleration_.setZero();
      reference_ = f.q;
    }
    if (finiteMotion(state_.mode)) {
      if (now - motion_at_ >= duration_) {
        Vec tcp_error = poseError(state_.tcp, finite_pose_target_);
        bool reached = (f.q - reference_).cwiseAbs().maxCoeff() <
                       config_.position_tolerance;
        if (state_.mode == Mode::PoseGoal || state_.mode == Mode::CartesianPath)
          reached &=
              tcp_error.head<3>().norm() < config_.tcp_position_tolerance &&
              tcp_error.tail<3>().norm() < config_.tcp_orientation_tolerance;
        if (reached &&
            f.dq.cwiseAbs().maxCoeff() < config_.velocity_tolerance) {
          if (settled_at_ == 0)
            settled_at_ = now;
          if (now - settled_at_ > 0.1) {
            event(state_.active_id, Outcome::Succeeded,
                  "measured target reached");
            state_.active_id = 0;
            state_.mode = Mode::Holding;
            ++generation_;
          }
        } else
          settled_at_ = 0;
        if (now - motion_at_ > duration_ + config_.settle_timeout) {
          fault("measured target settle timeout");
          return out;
        }
      }
    }
    Vec torque;
    if (state_.mode == Mode::Impedance) {
      const double alpha =
          1 - std::exp(-dt / config_.impedance_gain_time_constant);
      stiffness_ += alpha * (config_.stiffness - stiffness_);
      damping_ += alpha * (config_.damping - damping_);
      Jacobian J = model_.jacobian(f.q);
      Vec e = poseError(state_.tcp, impedance_reference_);
      if (e.head<3>().norm() > 0.1)
        e.head<3>() *= 0.1 / e.head<3>().norm();
      if (e.tail<3>().norm() > 0.25)
        e.tail<3>() *= 0.25 / e.tail<3>().norm();
      torque = J.transpose() *
                   (stiffness_.cwiseProduct(e) +
                    damping_.cwiseProduct(impedance_velocity_ - J * f.dq)) +
               model_.inverseDynamics(f.q, f.dq, Vec::Zero());
    } else {
      // Computed-torque position tracking: gains have units s^-2 and s^-1.
      // RNEA includes M(q), gravity and Coriolis exactly once; MCU arm Kp/Kd
      // are zero.
      Vec desired_acc = acceleration_ +
                        config_.kp.cwiseProduct(reference_ - f.q) +
                        config_.kd.cwiseProduct(velocity_ - f.dq);
      torque = model_.inverseDynamics(f.q, f.dq, desired_acc);
    }
    if (!torque.allFinite()) {
      fault("nonfinite torque");
      return out;
    }
    torque = torque.cwiseMax(-config_.torque).cwiseMin(config_.torque);
    Vec delta = config_.torque_rate * std::min(dt, config_.period);
    out.tau =
        torque.cwiseMax(last_torque_ - delta).cwiseMin(last_torque_ + delta);
    last_torque_ = out.tau;
    out.enabled = true;
    out.q = reference_;
    out.dq = velocity_;
    if (f.has_gripper) {
      double step = 0.4 * config_.period;
      double target = gripper_requested_ ? gripper_target_ : f.gripper_q;
      out.gripper_q =
          f.gripper_q + std::clamp(target - f.gripper_q, -step, step);
      out.gripper_kp = 8;
      out.gripper_kd = 0.5;
    }
    state_.compute_time = seconds(began);
    if (state_.compute_time > config_.period) {
      ++state_.overruns;
      fault("compute deadline exceeded");
      out.enabled = false;
      out.tau.setZero();
    }
  } catch (const std::exception &e) {
    fault(std::string("control exception: ") + e.what());
    out.enabled = false;
    out.tau.setZero();
  }
  return out;
}
Snapshot Controller::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}
std::vector<Event> Controller::events() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<Event> out(events_.begin(), events_.end());
  events_.clear();
  return out;
}
} // namespace z1
