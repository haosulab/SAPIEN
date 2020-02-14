#include "PS3_publisher.h"
namespace sapien::ros2 {

void PS3Publisher::pubPS3() {
  ps3->saveCache();
  auto axisData = ps3->exportAxisStates();
  auto buttonData = ps3->exportButtonStates();

  // swap for joy message convention
  axisData.push_back(*axisData.erase(axisData.begin() + 2));

  auto message = sensor_msgs::msg::Joy();

  // copy one by one since different type
  for (auto i : axisData) {
    message.axes.push_back(i);
  }
  for (auto i : buttonData) {
    message.buttons.push_back(i);
  }
  message.header.set__stamp(mClock->now());

  mPub->publish(message);
}

PS3Publisher::PS3Publisher(const std::string &nameSpace, rclcpp::Node *node, rclcpp::Clock::SharedPtr clock, double pubFrequency)
    : mNode(node) {
  ps3 = std::make_unique<sapien::robot::PS3>();
  mClock = clock;

  // Create Publisher
  std::string prefix = nameSpace + "/PS3_joystick";
  mPub = mNode->create_publisher<sensor_msgs::msg::Joy>(prefix, rmw_qos_profile_default);

  // Create Timer
  auto _interval = static_cast<unsigned long long>(1 / pubFrequency * 1e6);
  auto interval = std::chrono::microseconds(_interval);
  auto pubCallBack = std::bind(&PS3Publisher::pubPS3, this);

  // TODO: Use passed in clock for timer
//  rclcpp::create_timer(node, std::move(clock), rclcpp::Duration(interval), pubCallBack);
  timer_ = mNode->create_wall_timer(interval, pubCallBack);
};

}