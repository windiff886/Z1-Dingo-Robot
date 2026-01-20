#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <XmlRpcValue.h>

#include <fcntl.h>
#include <linux/input.h>
#include <termios.h>
#include <unistd.h>

#include <dirent.h>
#include <errno.h>
#include <cstring>

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

double XmlRpcToDouble(const XmlRpc::XmlRpcValue& v)
{
  if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(v);
  }
  if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(v);
  }
  return 0.0;
}

int XmlRpcToInt(const XmlRpc::XmlRpcValue& v)
{
  if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(v);
  }
  if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<int>(static_cast<double>(v));
  }
  return 0;
}

}  // namespace

class KeyboardCmdVel
{
public:
  KeyboardCmdVel()
  : nh_(),
    pnh_("~"),
    rate_hz_(20.0),
    key_hold_timeout_(0.5),
    require_enable_button_(true),
    enable_button_(-1),
    enable_turbo_button_(-1),
    axis_fb_(1),
    axis_lr_(0),
    axis_strafe_(-1),
    button_enable_(4),
    button_turbo_(5),
    num_axes_(6),
    num_buttons_(12),
    axis_linear_x_(-1),
    axis_linear_y_(-1),
    axis_angular_z_(-1),
    scale_linear_x_(0.0),
    scale_linear_y_(0.0),
    scale_linear_turbo_x_(0.0),
    scale_linear_turbo_y_(0.0),
    scale_angular_z_(0.0),
    scale_angular_turbo_z_(0.0),
    use_evdev_(false),
    input_fd_(-1),
    termios_ok_(false)
  {
    LoadParams();
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    InitInput();
  }

  ~KeyboardCmdVel()
  {
    PublishZero();
    if (input_fd_ >= 0) {
      close(input_fd_);
    }
    RestoreStdin();
  }

  void Run()
  {
    ros::Rate rate(rate_hz_);
    while (ros::ok()) {
      UpdatePressedKeys();
      geometry_msgs::Twist twist = ComputeTwist();
      cmd_pub_.publish(twist);
      rate.sleep();
    }
  }

private:
  struct KeyBindings {
    std::string forward;
    std::string backward;
    std::string left;
    std::string right;
    std::string strafe_left;
    std::string strafe_right;
    std::string enable;
    std::string turbo;
  } keys_;

  void LoadParams()
  {
    pnh_.param("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
    pnh_.param("input_device", input_device_, std::string(""));
    pnh_.param("key_hold_timeout", key_hold_timeout_, 0.5);

    pnh_.param("keyboard_to_joy/keys/forward", keys_.forward, std::string("w"));
    pnh_.param("keyboard_to_joy/keys/backward", keys_.backward, std::string("s"));
    pnh_.param("keyboard_to_joy/keys/left", keys_.left, std::string("a"));
    pnh_.param("keyboard_to_joy/keys/right", keys_.right, std::string("d"));
    pnh_.param("keyboard_to_joy/keys/strafe_left", keys_.strafe_left, std::string("q"));
    pnh_.param("keyboard_to_joy/keys/strafe_right", keys_.strafe_right, std::string("e"));
    pnh_.param("keyboard_to_joy/keys/enable", keys_.enable, std::string(" "));
    pnh_.param("keyboard_to_joy/keys/turbo", keys_.turbo, std::string("shift"));

    pnh_.param("keyboard_to_joy/axis_map/forward_backward", axis_fb_, 1);
    pnh_.param("keyboard_to_joy/axis_map/left_right", axis_lr_, 0);
    pnh_.param("keyboard_to_joy/axis_map/strafe", axis_strafe_, -1);

    pnh_.param("keyboard_to_joy/button_map/enable", button_enable_, 4);
    pnh_.param("keyboard_to_joy/button_map/turbo", button_turbo_, 5);

    pnh_.param("keyboard_to_joy/num_axes", num_axes_, 6);
    pnh_.param("keyboard_to_joy/num_buttons", num_buttons_, 12);
    pnh_.param("keyboard_to_joy/rate", rate_hz_, 20.0);

    pnh_.param("teleop_twist_joy/require_enable_button", require_enable_button_, true);
    pnh_.param("teleop_twist_joy/enable_button", enable_button_, 4);
    pnh_.param("teleop_twist_joy/enable_turbo_button", enable_turbo_button_, 5);

    LoadAxisAndScale();
  }

  void LoadAxisAndScale()
  {
    XmlRpc::XmlRpcValue v;

    axis_linear_x_ = -1;
    axis_linear_y_ = -1;
    if (pnh_.getParam("teleop_twist_joy/axis_linear", v)) {
      if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        axis_linear_x_ = XmlRpcToInt(v);
      } else if (v.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        if (v.hasMember("x")) {
          axis_linear_x_ = XmlRpcToInt(v["x"]);
        }
        if (v.hasMember("y")) {
          axis_linear_y_ = XmlRpcToInt(v["y"]);
        }
      }
    }

    axis_angular_z_ = -1;
    if (pnh_.getParam("teleop_twist_joy/axis_angular", v)) {
      if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        axis_angular_z_ = XmlRpcToInt(v);
      } else if (v.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        if (v.hasMember("yaw")) {
          axis_angular_z_ = XmlRpcToInt(v["yaw"]);
        }
      }
    }

    scale_linear_x_ = 0.0;
    scale_linear_y_ = 0.0;
    if (pnh_.getParam("teleop_twist_joy/scale_linear", v)) {
      if (v.getType() == XmlRpc::XmlRpcValue::TypeInt || v.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        scale_linear_x_ = XmlRpcToDouble(v);
      } else if (v.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        if (v.hasMember("x")) {
          scale_linear_x_ = XmlRpcToDouble(v["x"]);
        }
        if (v.hasMember("y")) {
          scale_linear_y_ = XmlRpcToDouble(v["y"]);
        }
      }
    }

    scale_linear_turbo_x_ = scale_linear_x_;
    scale_linear_turbo_y_ = scale_linear_y_;
    if (pnh_.getParam("teleop_twist_joy/scale_linear_turbo", v)) {
      if (v.getType() == XmlRpc::XmlRpcValue::TypeInt || v.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        scale_linear_turbo_x_ = XmlRpcToDouble(v);
      } else if (v.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        if (v.hasMember("x")) {
          scale_linear_turbo_x_ = XmlRpcToDouble(v["x"]);
        }
        if (v.hasMember("y")) {
          scale_linear_turbo_y_ = XmlRpcToDouble(v["y"]);
        }
      }
    }

    scale_angular_z_ = 0.0;
    if (pnh_.getParam("teleop_twist_joy/scale_angular", v)) {
      if (v.getType() == XmlRpc::XmlRpcValue::TypeInt || v.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        scale_angular_z_ = XmlRpcToDouble(v);
      } else if (v.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        if (v.hasMember("yaw")) {
          scale_angular_z_ = XmlRpcToDouble(v["yaw"]);
        }
      }
    }

    scale_angular_turbo_z_ = scale_angular_z_;
    if (pnh_.getParam("teleop_twist_joy/scale_angular_turbo", v)) {
      if (v.getType() == XmlRpc::XmlRpcValue::TypeInt || v.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        scale_angular_turbo_z_ = XmlRpcToDouble(v);
      } else if (v.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        if (v.hasMember("yaw")) {
          scale_angular_turbo_z_ = XmlRpcToDouble(v["yaw"]);
        }
      }
    }
  }

  void InitInput()
  {
    if (input_device_.empty()) {
      input_device_ = FindKeyboardDevice();
    }

    if (!input_device_.empty()) {
      input_fd_ = open(input_device_.c_str(), O_RDONLY | O_NONBLOCK);
      if (input_fd_ >= 0) {
        use_evdev_ = true;
        ROS_INFO_STREAM("Using input device: " << input_device_);
        return;
      }
      ROS_WARN_STREAM("Failed to open input device: " << input_device_ << " error=" << strerror(errno));
    }

    use_evdev_ = false;
    SetupStdin();
    ROS_WARN("Using stdin for keyboard input. For full key press/release support set ~input_device.");
  }

  void SetupStdin()
  {
    if (tcgetattr(STDIN_FILENO, &orig_termios_) != 0) {
      return;
    }
    termios raw = orig_termios_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
      termios_ok_ = true;
    }
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags >= 0) {
      fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }
  }

  void RestoreStdin()
  {
    if (termios_ok_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
    }
  }

  std::string FindKeyboardDevice() const
  {
    const std::string dir_path = "/dev/input/by-id";
    DIR* dir = opendir(dir_path.c_str());
    if (!dir) {
      return std::string();
    }

    std::string result;
    dirent* entry = nullptr;
    while ((entry = readdir(dir)) != nullptr) {
      std::string name = entry->d_name;
      if (name == "." || name == "..") {
        continue;
      }
      if (name.find("kbd") != std::string::npos) {
        result = dir_path + "/" + name;
        break;
      }
    }
    closedir(dir);
    return result;
  }

  std::string KeyCodeToString(uint16_t code) const
  {
    switch (code) {
      case KEY_W: return "w";
      case KEY_S: return "s";
      case KEY_A: return "a";
      case KEY_D: return "d";
      case KEY_Q: return "q";
      case KEY_E: return "e";
      case KEY_SPACE: return " ";
      case KEY_LEFTSHIFT:
      case KEY_RIGHTSHIFT:
        return "shift";
      default:
        return std::string();
    }
  }

  void UpdatePressedKeys()
  {
    if (use_evdev_) {
      UpdatePressedKeysEvdev();
    } else {
      UpdatePressedKeysStdin();
    }
  }

  void UpdatePressedKeysEvdev()
  {
    if (input_fd_ < 0) {
      return;
    }

    input_event ev;
    ssize_t n = 0;
    while ((n = read(input_fd_, &ev, sizeof(ev))) == static_cast<ssize_t>(sizeof(ev))) {
      if (ev.type != EV_KEY) {
        continue;
      }
      std::string key = KeyCodeToString(ev.code);
      if (key.empty()) {
        continue;
      }
      if (ev.value == 0) {
        pressed_keys_.erase(key);
      } else {
        pressed_keys_.insert(key);
      }
    }
  }

  void UpdatePressedKeysStdin()
  {
    ros::Time now = ros::Time::now();
    char c = 0;
    while (read(STDIN_FILENO, &c, 1) == 1) {
      if (c >= 'A' && c <= 'Z') {
        std::string key(1, static_cast<char>(c + 32));
        last_press_[key] = now;
        last_press_["shift"] = now;
        continue;
      }
      if (c >= 'a' && c <= 'z') {
        std::string key(1, c);
        last_press_[key] = now;
        continue;
      }
      if (c == ' ') {
        last_press_[" "] = now;
        continue;
      }
    }

    pressed_keys_.clear();
    for (auto it = last_press_.begin(); it != last_press_.end(); ) {
      if ((now - it->second).toSec() > key_hold_timeout_) {
        it = last_press_.erase(it);
      } else {
        pressed_keys_.insert(it->first);
        ++it;
      }
    }
  }

  void BuildJoy(std::vector<float>* axes, std::vector<int>* buttons) const
  {
    axes->assign(num_axes_, 0.0f);
    buttons->assign(num_buttons_, 0);

    if (axis_fb_ >= 0 && axis_fb_ < static_cast<int>(axes->size())) {
      if (pressed_keys_.count(keys_.forward)) {
        (*axes)[axis_fb_] = 1.0f;
      } else if (pressed_keys_.count(keys_.backward)) {
        (*axes)[axis_fb_] = -1.0f;
      }
    }

    if (axis_lr_ >= 0 && axis_lr_ < static_cast<int>(axes->size())) {
      if (pressed_keys_.count(keys_.left)) {
        (*axes)[axis_lr_] = 1.0f;
      } else if (pressed_keys_.count(keys_.right)) {
        (*axes)[axis_lr_] = -1.0f;
      }
    }

    if (axis_strafe_ >= 0 && axis_strafe_ < static_cast<int>(axes->size())) {
      if (pressed_keys_.count(keys_.strafe_left)) {
        (*axes)[axis_strafe_] = 1.0f;
      } else if (pressed_keys_.count(keys_.strafe_right)) {
        (*axes)[axis_strafe_] = -1.0f;
      }
    }

    if (button_enable_ >= 0 && button_enable_ < static_cast<int>(buttons->size())) {
      if (pressed_keys_.count(keys_.enable)) {
        (*buttons)[button_enable_] = 1;
      }
    }

    if (button_turbo_ >= 0 && button_turbo_ < static_cast<int>(buttons->size())) {
      if (pressed_keys_.count(keys_.turbo)) {
        (*buttons)[button_turbo_] = 1;
      }
    }
  }

  static bool ButtonPressed(const std::vector<int>& buttons, int index)
  {
    if (index < 0 || index >= static_cast<int>(buttons.size())) {
      return false;
    }
    return buttons[index] != 0;
  }

  static float AxisValue(const std::vector<float>& axes, int index)
  {
    if (index < 0 || index >= static_cast<int>(axes.size())) {
      return 0.0f;
    }
    return axes[index];
  }

  geometry_msgs::Twist ComputeTwist() const
  {
    geometry_msgs::Twist twist;
    std::vector<float> axes;
    std::vector<int> buttons;
    BuildJoy(&axes, &buttons);

    bool turbo = ButtonPressed(buttons, enable_turbo_button_);
    bool enabled = false;
    if (turbo) {
      enabled = true;
    } else if (ButtonPressed(buttons, enable_button_)) {
      enabled = true;
    } else if (!require_enable_button_) {
      enabled = true;
    }

    if (!enabled) {
      return twist;
    }

    double linear_x_scale = turbo ? scale_linear_turbo_x_ : scale_linear_x_;
    double linear_y_scale = turbo ? scale_linear_turbo_y_ : scale_linear_y_;
    double angular_z_scale = turbo ? scale_angular_turbo_z_ : scale_angular_z_;

    if (axis_linear_x_ >= 0) {
      twist.linear.x = AxisValue(axes, axis_linear_x_) * linear_x_scale;
    }
    if (axis_linear_y_ >= 0) {
      twist.linear.y = AxisValue(axes, axis_linear_y_) * linear_y_scale;
    }
    if (axis_angular_z_ >= 0) {
      twist.angular.z = AxisValue(axes, axis_angular_z_) * angular_z_scale;
    }

    return twist;
  }

  void PublishZero()
  {
    if (!cmd_pub_) {
      return;
    }
    geometry_msgs::Twist twist;
    cmd_pub_.publish(twist);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher cmd_pub_;

  std::string cmd_vel_topic_;
  double rate_hz_;
  double key_hold_timeout_;
  bool require_enable_button_;
  int enable_button_;
  int enable_turbo_button_;

  int axis_fb_;
  int axis_lr_;
  int axis_strafe_;
  int button_enable_;
  int button_turbo_;
  int num_axes_;
  int num_buttons_;

  int axis_linear_x_;
  int axis_linear_y_;
  int axis_angular_z_;
  double scale_linear_x_;
  double scale_linear_y_;
  double scale_linear_turbo_x_;
  double scale_linear_turbo_y_;
  double scale_angular_z_;
  double scale_angular_turbo_z_;

  bool use_evdev_;
  int input_fd_;
  std::string input_device_;
  termios orig_termios_;
  bool termios_ok_;
  std::unordered_set<std::string> pressed_keys_;
  std::unordered_map<std::string, ros::Time> last_press_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyboard_cmd_vel");
  KeyboardCmdVel node;
  node.Run();
  return 0;
}
