#include <rviz/panel.h>
#include <pluginlib/class_list_macros.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <unidingo_rviz_test/PreviewStatus.h>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QTabWidget>
#include <QVBoxLayout>
#include <array>

namespace unidingo_rviz_test {
class TargetPanel : public rviz::Panel {
  Q_OBJECT
  struct Controls {
    QLabel *mode{}, *detail{}, *target{};
    QPushButton *send{};
  };
  std::array<Controls, 2> arms_;
  QTabWidget *tabs_{};
  ros::CallbackQueue queue_;
  ros::NodeHandle node_;
  ros::Publisher actions_;
  ros::Subscriber status_;

  QPushButton *button(const QString &text, const std::string &action,
                       QBoxLayout *layout, const QString &style = {}) {
    auto *result = new QPushButton(text);
    result->setMinimumHeight(29);
    result->setStyleSheet(style);
    connect(result, &QPushButton::clicked, this, [this, action] {
      std_msgs::String msg;
      msg.data = action;
      actions_.publish(msg);
    });
    layout->addWidget(result);
    return result;
  }
  void status(const PreviewStatus::ConstPtr &msg) {
    if (msg->arm != "left" && msg->arm != "right")
      return;
    const int index = msg->arm == "left" ? 0 : 1;
    auto &ui = arms_[index];
    tabs_->setTabText(index, (index == 0 ? "LEFT · " : "RIGHT · ") +
                              QString::fromStdString(msg->ready ? "Ready" : msg->mode));
    ui.mode->setText(QString::fromStdString(msg->mode));
    ui.detail->setText(QString::fromStdString(msg->detail));
    ui.send->setEnabled(msg->ready);
    if (msg->target_available) {
      const auto &p = msg->target_pose;
      ui.target->setText(QString("%1 · %2\nx %3   y %4   z %5 m\nq xyzw [%6, %7, %8, %9]")
          .arg(msg->editing ? "TARGET" : "MEASURED")
          .arg(QString::fromStdString(msg->header.frame_id))
          .arg(p.position.x, 0, 'f', 4).arg(p.position.y, 0, 'f', 4).arg(p.position.z, 0, 'f', 4)
          .arg(p.orientation.x, 0, 'f', 3).arg(p.orientation.y, 0, 'f', 3)
          .arg(p.orientation.z, 0, 'f', 3).arg(p.orientation.w, 0, 'f', 3));
    } else {
      ui.target->setText("Waiting for target pose");
    }
  }

public:
  explicit TargetPanel(QWidget *parent = nullptr) : rviz::Panel(parent) {
    node_.setCallbackQueue(&queue_);
    actions_ = node_.advertise<std_msgs::String>("/rviz_tcp_test/ui_action", 10);
    status_ = node_.subscribe("/rviz_tcp_test/ui_status", 10, &TargetPanel::status, this);
    setMinimumWidth(310);
    auto *layout = new QVBoxLayout(this);
    auto *hint = new QLabel("Drag a target → preview → send");
    hint->setWordWrap(true);
    layout->addWidget(hint);
    button("HOME TARGETS · both arms", "both/home", layout,
           "QPushButton {background:#d6edf8; color:#183b53; font-weight:bold;}");
    tabs_ = new QTabWidget();
    connect(tabs_, &QTabWidget::currentChanged, this, [this](int index) {
      std_msgs::String msg;
      msg.data = index == 0 ? "left/select" : "right/select";
      actions_.publish(msg);
    });
    layout->addWidget(tabs_);
    for (size_t i = 0; i < arms_.size(); ++i) {
      const std::string side = i == 0 ? "left" : "right";
      auto *group = new QGroupBox();
      auto *body = new QVBoxLayout(group);
      body->setSpacing(5);
      auto &ui = arms_[i];
      ui.mode = new QLabel("Waiting for feedback");
      ui.mode->setStyleSheet("font-weight:bold; color:#34536b;");
      ui.detail = new QLabel("Waiting for preview node");
      ui.detail->setWordWrap(true);
      ui.detail->setMinimumHeight(32);
      ui.detail->setTextFormat(Qt::PlainText);
      ui.target = new QLabel("Waiting for target pose");
      ui.target->setWordWrap(true);
      ui.target->setTextFormat(Qt::PlainText);
      ui.target->setStyleSheet("font-family:monospace; font-size:12px; background:#edf1f4; padding:5px;");
      body->addWidget(ui.mode);
      body->addWidget(ui.detail);
      body->addWidget(ui.target);
      auto *presets = new QHBoxLayout();
      button("Home pose", side + "/home", presets);
      button("Measured TCP", side + "/reset", presets);
      body->addLayout(presets);
      ui.send = button("SEND POSE", side + "/send", body,
          "QPushButton:enabled {background:#24794b; color:white; font-weight:bold;}"
          "QPushButton:disabled {color:#777; background:#ddd;}");
      connect(ui.send, &QPushButton::clicked, ui.send, [send = ui.send] { send->setEnabled(false); });
      ui.send->setMinimumHeight(34);
      ui.send->setEnabled(false);
      auto *stop = new QHBoxLayout();
      button("STOP", side + "/stop", stop, "color:#a92323; font-weight:bold;");
      button("Hold after stop", side + "/hold", stop);
      body->addLayout(stop);
      auto *operations = new QHBoxLayout();
      button("Enable", side + "/enable", operations);
      button("Disable", side + "/disable", operations);
      button("Ack fault", side + "/resume", operations);
      body->addLayout(operations);
      tabs_->addTab(group, i == 0 ? "LEFT" : "RIGHT");
    }
    auto *legend = new QLabel("Solid: feedback\nOrange: MoveIt target · Violet: SDK reference");
    legend->setWordWrap(true);
    layout->addWidget(legend);
    layout->addStretch();
    auto *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this] { queue_.callAvailable(ros::WallDuration(0)); });
    timer->start(50);
  }
};
} // namespace unidingo_rviz_test
PLUGINLIB_EXPORT_CLASS(unidingo_rviz_test::TargetPanel, rviz::Panel)
#include "target_panel.moc"
