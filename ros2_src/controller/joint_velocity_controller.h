#pragma once

#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sapien_ros2_communication_interface/srv/joint_velocity.hpp"
#include "spdlog/spdlog.h"
#include "utils/thread_safe_structure.hpp"

#define HANDLE_COMMAND(continuous, vec)                                                           \
  {                                                                                               \
    if (continuous) {                                                                             \
      mContinuousCommands.write(vec);                                                             \
    } else {                                                                                      \
      mCommands.push(vec);                                                                        \
    }                                                                                             \
  }

namespace sapien::ros2 {

class SControllableArticulationWrapper;
class RobotManager;

class JointVelocityController {
  friend RobotManager;

public:
protected:
  rclcpp::Node::SharedPtr mNode;
  rclcpp::Clock::SharedPtr mClock;
  rclcpp::Service<sapien_ros2_communication_interface::srv::JointVelocity>::SharedPtr mService;

  std::vector<std::string> mJointNames;

  ThreadSafeVector<float> mContinuousCommands;
  ThreadSafeQueue<std::vector<float>> mCommands;

public:
  JointVelocityController(const std::string &nameSpace, rclcpp::Node::SharedPtr node,
                          rclcpp::Clock::SharedPtr clock,
                          SControllableArticulationWrapper *wrapper,
                          const std::vector<std::string> &jointNames,
                          const std::string &serviceName);;

  void moveJoint(const std::vector<float> &velocity, bool continuous = false);;

  void moveJoint(const std::vector<std::string> &jointNames, const std::vector<float> &velocity,
                 bool continuous = false);;

  void moveJoint(const std::vector<std::string> &jointNames, float velocity,
                 bool continuous = false);;

protected:
  void handleService(
      const std::shared_ptr<sapien_ros2_communication_interface::srv::JointVelocity_Request> req,
      std::shared_ptr<sapien_ros2_communication_interface::srv::JointVelocity_Response> res);
};
}; // namespace sapien::ros2
