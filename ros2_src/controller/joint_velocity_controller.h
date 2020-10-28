#pragma once

#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sapien_ros2_communication_interface/srv/joint_velocity.hpp"
#include "spdlog/spdlog.h"
#include "utils/delayed_controller_base.hpp"


namespace sapien::ros2 {

class SControllableArticulationWrapper;
class RobotManager;

class JointVelocityController : public DelayedControllerBase {
  friend RobotManager;

public:
protected:
  rclcpp::Node::SharedPtr mNode;
  rclcpp::Service<sapien_ros2_communication_interface::srv::JointVelocity>::SharedPtr mService;

  std::vector<std::string> mJointNames;

  ThreadSafeVector<float> mContinuousCommands;
  ThreadSafeQueue<std::vector<float>> mCommands;

public:
  JointVelocityController(rclcpp::Node::SharedPtr node, rclcpp::Clock::SharedPtr clock,
                          sapien::ros2::SControllableArticulationWrapper *wrapper,
                          const std::vector<std::string> &jointNames,
                          const std::string &serviceName="", double latency=0);

  void moveJoint(const std::vector<float> &velocity, bool continuous = false);

  void moveJoint(const std::vector<std::string> &jointNames, const std::vector<float> &velocity,
                 bool continuous = false);

  void moveJoint(const std::vector<std::string> &jointNames, float velocity,
                 bool continuous = false);

protected:
  void handleService(
      const std::shared_ptr<sapien_ros2_communication_interface::srv::JointVelocity_Request> req,
      std::shared_ptr<sapien_ros2_communication_interface::srv::JointVelocity_Response> res);
};
} // namespace sapien::ros2
