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
  rclcpp::Node::SharedPtr mNode;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mPub;
  sensor_msgs::msg::JointState *mJointStates = nullptr;

  rclcpp::Clock::SharedPtr mClock;
  rclcpp::Duration mInterval;
  std::thread mThread;
  bool stop = false;

public:
  JointPublisher(const std::string &nameSpace, const rclcpp::Node::SharedPtr& node,
                 rclcpp::Clock::SharedPtr clock, sensor_msgs::msg::JointState *states,
                 double pubFrequency);
  ~JointPublisher() = default;
  ;

protected:
  void publishLoop();
  inline void start() { mThread = std::thread(&JointPublisher::publishLoop, this); }
};

} // namespace sapien::ros2
