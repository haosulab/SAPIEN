//
// Created by zack on 2/21/20.
//
#include "urdf/ros_urdf_loader.h"

using namespace sapien::ros2;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("ros_test");

  new ROS_urdf_Loaer(node, "../ros2_ws/src/sapien_resources/panda_urdf_description/urdf/panda.urdf");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}