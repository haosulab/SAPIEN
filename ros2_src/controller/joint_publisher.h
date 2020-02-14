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
  JointPublisher(const std::string &nameSpace, rclcpp::Node::SharedPtr node,
                 rclcpp::Clock::SharedPtr clock, sensor_msgs::msg::JointState *states,
                 double pubFrequency)
      : mNode(node), mJointStates(states), mClock(std::move(clock)),
        mInterval(
            std::chrono::microseconds(static_cast<unsigned long long>(1 / pubFrequency * 1e6))) {
    // Create Publisher
    std::string prefix = nameSpace + "/joint_states";
    mPub = node->create_publisher<sensor_msgs::msg::JointState>(prefix, rmw_qos_profile_default);
    //    auto pubCallBack = std::bind(&JointPublisher::pubJointStates, this);

    //    rclcpp::create_timer(node, std::move(clock), mInterval, pubCallBack);
  };
  ~JointPublisher() = default;
  ;

protected:
  void publishLoop() {
    auto target = mClock->now() + mInterval;
    while (rclcpp::ok() && !stop) {
      if (mClock->now() > target) {
        mPub->publish(*mJointStates);
        RCLCPP_INFO(mNode->get_logger(), "1");
        target = target + mInterval;
      }
    }
  };
  void start() { mThread = std::thread(&JointPublisher::publishLoop, this); }
};

} // namespace sapien::ros2
