#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <pluginlib/class_list_macros.h>
#include <rviz/panel_dock_widget.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/status_property.h>
#include <unidingo_rviz_test/EditorCommand.h>
#include <unidingo_rviz_test/EditorTarget.h>
#include <QAction>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace unidingo_rviz_test {
// Keep MoveIt's editor, IK, goal robot and scene-frame trajectory playback.
// Only the SDK-specific presets, target messages and preview reset live here.
class SdkMotionPlanning : public moveit_rviz_plugin::MotionPlanningDisplay {
  Q_OBJECT
  std::array<ros::Publisher, 2> targets_;
  ros::Subscriber actions_;
  std::array<std::vector<double>, 2> published_;
  std::array<int, 2> flags_{{-1, -1}};
  std::array<std::string, 2> tcp_frames_;
  std::array<uint64_t, 2> revisions_{{0, 0}};

  void hideExecutionPanel() {
    frame_->setEnabled(false);
    if (frame_dock_) {
      frame_dock_->hide();
      frame_dock_->toggleViewAction()->setVisible(false);
    }
  }

  void publishTargets(int acknowledge = -1) {
    if (!query_goal_state_)
      return;
    const auto state = getQueryGoalState();
    for (int i = 0; i < 2; ++i) {
      const std::string side = i == 0 ? "left" : "right";
      const std::string group = side + "_arm";
      const bool has_tcp = getRobotModel()->hasLinkModel(tcp_frames_[i]);
      setStatus(has_tcp ? rviz::StatusProperty::Ok : rviz::StatusProperty::Error,
                QString::fromStdString(side + " TCP"),
                QString::fromStdString(has_tcp ? tcp_frames_[i] : "Missing display link: " + tcp_frames_[i]));
      std::vector<double> q;
      state->copyJointGroupPositions(group, q);
      bool valid = has_tcp;
      for (const auto &eef : robot_interaction_->getActiveEndEffectors())
        if (eef.parent_group == group && query_goal_state_->inError(eef))
          valid = false;
      const bool editing = modified_groups_.count(group);
      const int flags = int(editing) + 2 * int(valid);
      const bool changed = q != published_[i] || flags != flags_[i];
      if (!changed && acknowledge != i)
        continue;
      if (changed) {
        published_[i] = q;
        flags_[i] = flags;
        ++revisions_[i];
      }
      EditorTarget msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = side + "_link00";
      msg.arm = side;
      msg.revision = revisions_[i];
      msg.editing = editing;
      msg.valid = valid;
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      if (has_tcp)
        pose = state->getGlobalLinkTransform(msg.header.frame_id).inverse() *
               state->getGlobalLinkTransform(tcp_frames_[i]);
      const Eigen::Quaterniond orientation(pose.linear());
      msg.pose.position.x = pose.translation().x();
      msg.pose.position.y = pose.translation().y();
      msg.pose.position.z = pose.translation().z();
      msg.pose.orientation.x = orientation.x();
      msg.pose.orientation.y = orientation.y();
      msg.pose.orientation.z = orientation.z();
      msg.pose.orientation.w = orientation.w();
      targets_[i].publish(msg);
    }
  }

  void action(const EditorCommand &command) {
    if (!query_goal_state_)
      return;
    if (command.operation == "clear_path") {
      trajectory_visual_->reset();
      return;
    }
    const auto &side = command.arm;
    const auto &operation = command.operation;
    if (side != "left" && side != "right")
      return;
    // Flush an edit already applied by the native handler before resetting a
    // completed command's goal. A new edit belongs to the next command.
    publishTargets();
    if (command.expected_revision && command.expected_revision != revisions_[side == "left" ? 0 : 1])
      return;
    const std::string group = side + "_arm";
    if (operation == "select") {
      const bool edited = modified_groups_.count(group);
      changePlanningGroup(group);
      // Merely switching tabs must not turn a following arm into an edited goal.
      if (!edited)
        modified_groups_.erase(group);
      trajectory_visual_->reset();
    } else if (operation == "home" || operation == "reset") {
      moveit::core::RobotState state(*getQueryGoalState());
      const auto *jmg = state.getJointModelGroup(group);
      if (operation == "home") {
        state.setToDefaultValues(jmg, "home");
        modified_groups_.insert(group);
      } else {
        const auto scene = getPlanningSceneRO();
        std::vector<double> measured;
        scene->getCurrentState().copyJointGroupPositions(jmg, measured);
        state.setJointGroupPositions(jmg, measured);
        modified_groups_.erase(group);
      }
      state.update();
      query_goal_state_->clearError();
      setQueryGoalState(state);
    }
    publishTargets(side == "left" ? 0 : 1);
  }

protected:
  void onInitialize() override {
    moveit_rviz_plugin::MotionPlanningDisplay::onInitialize();
    targets_[0] = update_nh_.advertise<EditorTarget>("/rviz_tcp_test/left/editor_target", 1, true);
    targets_[1] = update_nh_.advertise<EditorTarget>("/rviz_tcp_test/right/editor_target", 1, true);
    for (int i = 0; i < 2; ++i) {
      const std::string side = i == 0 ? "left" : "right";
      std::string frame;
      update_nh_.param<std::string>("/" + side + "_arm/controller/tool_frame", frame, "tool");
      tcp_frames_[i] = side + "_" + frame;
    }
    actions_ = update_nh_.subscribe<EditorCommand>("/rviz_tcp_test/editor_action", 10,
        [this](const EditorCommand::ConstPtr &msg) {
          addMainLoopJob([this, command = *msg] { action(command); });
        });
    connect(this, &moveit_rviz_plugin::MotionPlanningDisplay::queryGoalStateChanged,
            this, [this] { addMainLoopJob([this] { publishTargets(); }); });
    // Execution belongs to PoseGoal. Do not expose MoveIt's separate execution UI.
    hideExecutionPanel();
  }
  void onRobotModelLoaded() override {
    moveit_rviz_plugin::MotionPlanningDisplay::onRobotModelLoaded();
    modified_groups_.clear();
    published_ = {};
    flags_ = {{-1, -1}};
    query_goal_state_->setUpdateCallback(
        [this](robot_interaction::InteractionHandler *handler, bool error_changed) {
          // Preserve the upstream update path; enqueue only the SDK adapter work.
          scheduleDrawQueryGoalState(handler, error_changed);
          addMainLoopJob([this] {
            modified_groups_.insert(getCurrentPlanningGroup());
            publishTargets();
          });
        });
  }
  void onEnable() override {
    moveit_rviz_plugin::MotionPlanningDisplay::onEnable();
    hideExecutionPanel();
  }
public:
  ~SdkMotionPlanning() override {
    actions_.shutdown();
    if (query_goal_state_)
      query_goal_state_->setUpdateCallback({});
    clearJobs();
  }
  void reset() override {
    moveit_rviz_plugin::MotionPlanningDisplay::reset();
    hideExecutionPanel();
  }
};
} // namespace unidingo_rviz_test
PLUGINLIB_EXPORT_CLASS(unidingo_rviz_test::SdkMotionPlanning, rviz::Display)
#include "sdk_motion_planning.moc"
