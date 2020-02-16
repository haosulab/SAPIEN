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
  std::shared_ptr<rclcpp::Node> mNode;
  rclcpp::Clock::SharedPtr mClock;
  std::unique_ptr<sapien::ros2::PS3> ps3;
  rclcpp::TimerBase::SharedPtr timer;

private:
  void pubPS3();

public:
  PS3Publisher(const std::string &nameSpace, std::shared_ptr<rclcpp::Node> node, rclcpp::Clock::SharedPtr clock, double pubFrequency);
};

} // namespace sapien::ros2
