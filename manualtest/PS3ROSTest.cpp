//
// Created by zack on 2/10/20.
//
#include "device/PS3_publisher.h"

using namespace sapien::ros2;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto clock = rclcpp::Clock::make_shared(RCL_ROS_TIME);
  auto node = std::make_shared<rclcpp::Node>("ros_test");
  //  auto clock = node->get_clock();

  new PS3Publisher("ros_test", node.get(), clock, 20.0f);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}