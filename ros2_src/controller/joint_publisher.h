#pragma once

#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>
#include <functional>
#include <string>
#include <utility>

namespace sapien::ros2 {

class JointPublisher {
  friend class RobotManager;

protected:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mPub;
  rclcpp::Node *mNode;
  rclcpp::Clock::SharedPtr mClock;

  sensor_msgs::msg::JointState *mJointStates = nullptr;

protected:
  JointPublisher(const std::string &nameSpace, rclcpp::Node *node, rclcpp::Clock::SharedPtr clock,
                 sensor_msgs::msg::JointState *states, double pubFrequency)
      : mNode(node), mJointStates(states) {
    // Create Timer
    auto _interval = static_cast<unsigned long long>(1 / pubFrequency * 1e6);
    auto interval = std::chrono::microseconds(_interval);
    auto pubCallBack = std::bind(&JointPublisher::pubJointStates, this);
    rclcpp::create_timer(node, std::move(clock), rclcpp::Duration(interval), pubCallBack);

    // Create Publisher
    std::string prefix = nameSpace + "/joint_states";
    mPub = node->create_publisher<sensor_msgs::msg::JointState>(prefix, 5);
  };


  void pubJointStates() { mPub->publish(*mJointStates); };
};

} // namespace sapien::ros2
