#pragma once

#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sapien_ros2_communication_interface/srv/joint_velocity.hpp"
#include "utils/thread_safe_structure.hpp"

namespace sapien::ros2 {

class SControllableArticulationWrapper;

class JointVelocityController {
public:
protected:
  rclcpp::Node::SharedPtr mNode;
  rclcpp::Clock::SharedPtr mClock;

  std::vector<std::string> mJointNames;

  ThreadSafeVector<float> mContinuousCommands;
  ThreadSafeQueue<std::vector<float>> mCommands;

public:
protected:
  JointVelocityController(const std::string &nameSpace, const rclcpp::Node::SharedPtr &node,
                          rclcpp::Clock::SharedPtr clock,
                          SControllableArticulationWrapper *wrapper,
                          const std::vector<std::string> &jointNames,
                          const std::string &serviceName)
      : mNode(node), mClock(std::move(clock)), mJointNames(jointNames),
        mContinuousCommands(jointNames.size()), mCommands() {
    auto serverName = nameSpace + "/" + serviceName;
    //    mNode->
  };

  void handleService(
      const std::shared_ptr<sapien_ros2_communication_interface::srv::JointVelocity_Request> req,
      std::shared_ptr<sapien_ros2_communication_interface::srv::JointVelocity_Response> res) {
    std::vector<float> jointVelocity = req->velocity;
    if (req->name.empty()) {
      if (jointVelocity.size() != mJointNames.size()) {
        RCLCPP_WARN(mNode->get_logger(),
                    "Joint Velocity Server receive service request with %i velocity, not same as "
                    "joint numbers %i",
                    jointVelocity.size(), mJointNames.size());
        res->success = false;
        return;
      }
      if(req->continuous){

      }
    }
  };
};
}