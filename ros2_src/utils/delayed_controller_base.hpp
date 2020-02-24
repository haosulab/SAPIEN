#pragma once
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "utils/thread_safe_structure.hpp"

namespace sapien::ros2 {

class DelayedControllerBase {
protected:
  rclcpp::Clock::SharedPtr mClock;
  rclcpp::Duration mLatency;
  std::vector<ThreadSafeQueue<rclcpp::Time>> mCommandTimer;

protected:
  inline DelayedControllerBase(rclcpp::Clock::SharedPtr clock, double latency,
                               int numControllers = 1)
      : mClock(std::move(clock)), mLatency(latency * 1e9), mCommandTimer(numControllers){
  };

  inline void updateCommandTimer(const rclcpp::Time &commandedTime, int num = 0) {
    mCommandTimer[num].push(commandedTime + mLatency);
  }
};
}