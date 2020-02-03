#pragma once

#include <utility>

#include "joint_publisher.h"
#include "rclcpp/rclcpp.hpp"

namespace sapien::ros2 {

class JointPublisher;
class SceneManager;
class ControllableArticulationWrapper;
class RobotManager {
  friend SceneManager;

public:
protected:
  std::unique_ptr<JointPublisher> mJointPublisher = nullptr;
  rclcpp::Node::UniquePtr mNode = nullptr;
  sensor_msgs::msg::JointState::UniquePtr mJointStates = nullptr;

  // Cache
  std::string mNameSpace;
  uint32_t mJointNum = 0;
  ControllableArticulationWrapper *wrapper;
  rclcpp::Clock::SharedPtr mClock;

public:
  const std::vector<std::string> getDriveJointNames(){};
  void createJointPublisher(double pubFrequency) {
    if (mJointPublisher) {
      RCLCPP_WARN(mNode->get_logger(),
                  "Joint Pub Node has already been created for this Robot Manager");
      RCLCPP_WARN(mNode->get_logger(), "Robot Manager will use the original joint state pub node");
      return;
    }

    mJointPublisher = std::make_unique<JointPublisher>(this, mNameSpace, &mNode, mClock,
                                                       mJointStates.get(), pubFrequency);
  };

protected:
  RobotManager(ControllableArticulationWrapper *wrapper, const std::string& nameSpace, rclcpp::Clock::SharedPtr clock)
      : wrapper(wrapper), mClock(std::move(clock)) {

    // Create Node
    auto nameIter = nameSpace.find('/');
    auto nodeName(nameSpace);
    nodeName[nameIter] = '_';
    mNode = rclcpp::Node::make_unique(nodeName, nameSpace);

    // Create Initial Joint States
    mJointStates = std::make_unique<sensor_msgs::msg::JointState>();
    auto jointName = wrapper->getDriveJointNames();
    mJointNum = jointName.size();
    mJointStates->name = jointName;
    mJointStates->position.resize(jointName.size());
    mJointStates->velocity.resize(jointName.size());
  };

  void updateJointStates(const std::vector<float> &jointAngles,
                         const std::vector<float> &jointVelocity) {
    mJointStates->header.stamp = mClock->now();
    mJointStates->position.assign(jointAngles.begin(), jointAngles.begin() + mJointNum);
    mJointStates->velocity.assign(jointVelocity.begin(), jointVelocity.begin() + mJointNum);
  };

  // Step function when simulator step once
  void step() {
    if (mJointPublisher) {
      updateJointStates(wrapper->popCurrentJointPositions, wrapper->popCurrentJointVelocities);
    }
  };
};
} // namespace sapien::ros2
