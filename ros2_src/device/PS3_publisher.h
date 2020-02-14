#pragma once

#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "joystick_ps3.h"
#include <chrono>
#include <functional>
#include <string>
#include <utility>
#include <unistd.h>

using namespace std::chrono_literals;

namespace sapien::ros2 {

class PS3Publisher{

protected:
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr mPub;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Node *mNode;
  rclcpp::Clock::SharedPtr mClock;
  std::unique_ptr<sapien::robot::PS3> ps3;

private:
  void pubPS3();

public:
  PS3Publisher(const std::string &nameSpace, rclcpp::Node *node, rclcpp::Clock::SharedPtr clock, double pubFrequency);
};

} // namespace sapien::ros2
