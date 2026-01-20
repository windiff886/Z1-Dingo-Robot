#include <nodelet/loader.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "d455_camera_node");
  nodelet::Loader loader;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  if (!loader.load(ros::this_node::getName(), "d455_camera/D455CameraNodelet", remap, nargv)) {
    ROS_ERROR("Failed to load d455_camera nodelet.");
    return 1;
  }
  ros::spin();
  return 0;
}
