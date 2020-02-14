#pragma once

#include <numeric>
#include <utility>

#include "joint_publisher.h"
#include "rclcpp/rclcpp.hpp"
#include "sapien_controllable_articulation.h"

namespace sapien::ros2 {

class JointPublisher;
class SceneManager;
class SControllableArticulation;
class RobotManager {
  friend class SceneManager;

public:
  SControllableArticulation *wrapper;

protected:
  // State
  rclcpp::Node::SharedPtr mNode;
  sensor_msgs::msg::JointState::UniquePtr mJointStates = nullptr;
  rclcpp::Clock::SharedPtr mClock;

  // Cache
  std::string mNameSpace;
  uint32_t mJointNum = 0;

  // Controllers
  std::unique_ptr<JointPublisher> mJointPublisher = nullptr;

public:
  void createJointPublisher(double pubFrequency) {
    if (mJointPublisher) {
      RCLCPP_WARN(mNode->get_logger(),
                  "Joint Pub Node has already been created for this Robot Manager");
      RCLCPP_WARN(mNode->get_logger(), "Robot Manager will use the original joint state pub node");
      return;
    }

    mJointPublisher = std::make_unique<JointPublisher>(mNameSpace, mNode->shared_from_this(),
                                                       mClock, mJointStates.get(), pubFrequency);
  };

  RobotManager(SControllableArticulation *wrapper, const std::string &nameSpace,
               const std::string &robotName, rclcpp::Clock::SharedPtr clock)
      : wrapper(wrapper), mClock(std::move(clock)), mNameSpace(nameSpace + "/" + robotName) {

    mNode = rclcpp::Node::make_shared(robotName, nameSpace);

    // Create Initial Joint States
    mJointStates = std::make_unique<sensor_msgs::msg::JointState>();
    auto jointName = wrapper->getDriveJointNames();
    mJointNum = jointName.size();
    mJointStates->name = jointName;
    mJointStates->position.resize(jointName.size());
    mJointStates->velocity.resize(jointName.size());
  };

  void setDriveProperty(float stiffness, float damping, float forceLimit = PX_MAX_F32,
                        const std::vector<uint32_t> &jointIndex = {}) {
    std::vector<uint32_t> index(jointIndex);
    if (jointIndex.empty()) {
      index.resize(wrapper->mJoints.size());
      std::iota(std::begin(index), std::end(index), 0);
    }

    for (unsigned int i : index) {
      assert(i < wrapper->mArticulation->dof());
      auto joint = wrapper->mJoints[i];
      joint->setDriveProperty(stiffness, damping, forceLimit);
    }
  };

  void balancePassiveForce(bool gravity = true, bool coriolisAndCentrifugal = true,
                           bool external = true) {
    auto balanceForce =
        wrapper->mArticulation->computePassiveForce(gravity, coriolisAndCentrifugal, external);
    wrapper->mArticulation->setQf(balanceForce);
  };

protected:
  void updateJointStates(const std::vector<float> &jointAngles,
                         const std::vector<float> &jointVelocity) {
    mJointStates->header.stamp = mClock->now();
    mJointStates->position.assign(jointAngles.begin(), jointAngles.begin() + mJointNum);
    mJointStates->velocity.assign(jointVelocity.begin(), jointVelocity.begin() + mJointNum);
    mJointStates->header.stamp = mClock->now();
  };

  // Step function when simulator step once
  void step() {
    updateJointStates(wrapper->mJointPositions.read(), wrapper->mJointVelocities.read());
  };

  void start() {
    if (mJointPublisher) {
      mJointPublisher->start();
    }
  }
};
} // namespace sapien::ros2
