#pragma once

#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
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
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mPub;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Node *mNode;
  std::unique_ptr<sapien::robot::PS3> ps3;

private:
  void pubPS3() {
    RCLCPP_INFO(mNode->get_logger(), "HERE");

    ps3->saveCache();
    int a = ps3->exportButtonStates()[0];
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(a);

    std::cout << "HERE\n";

    mPub->publish(message);
  }

public:
  PS3Publisher(const std::string &nameSpace, rclcpp::Node *node, rclcpp::Clock::SharedPtr clock, double pubFrequency)
      : mNode(node) {
    ps3 = std::make_unique<sapien::robot::PS3>();

    // Create Publisher
    std::string prefix = nameSpace + "/PS3_joystick";
    mPub = mNode->create_publisher<std_msgs::msg::String>(prefix, rmw_qos_profile_default);

    // Create Timer
    auto _interval = static_cast<unsigned long long>(1 / pubFrequency * 1e6);
    auto interval = std::chrono::microseconds(_interval);
    auto pubCallBack = std::bind(&PS3Publisher::pubPS3, this);

    // TODO: use correct clock
    // rclcpp::create_timer(node, std::move(clock), rclcpp::Duration(interval), pubCallBack);
    timer_ = mNode->create_wall_timer(interval, pubCallBack);
  };
};

} // namespace sapien::ros2
