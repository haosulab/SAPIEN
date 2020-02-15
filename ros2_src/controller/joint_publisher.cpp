#include "joint_publisher.h"

namespace sapien::ros2 {
JointPublisher::JointPublisher(const std::string &nameSpace, const rclcpp::Node::SharedPtr &node,
                               rclcpp::Clock::SharedPtr clock,
                               sensor_msgs::msg::JointState *states, double pubFrequency)
    : mNode(node), mJointStates(states), mClock(std::move(clock)),
      mInterval(
          std::chrono::microseconds(static_cast<unsigned long long>(1 / pubFrequency * 1e6))) {

  // Create Publisher
  std::string prefix = nameSpace + "/joint_states";
  mPub = node->create_publisher<sensor_msgs::msg::JointState>(prefix, 10);
  mTimer = rclcpp::create_timer(node, mClock, rclcpp::Duration(mInterval),
                                std::bind(&JointPublisher::publishLoop, this));
}
void JointPublisher::publishLoop() {
  mPub->publish(*mJointStates);
  RCLCPP_INFO(mNode->get_logger(), "1");
}
}