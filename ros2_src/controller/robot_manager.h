#pragma once

#include <numeric>
#include <utility>

#include "joint_velocity_controller.h"
#include "joint_publisher.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <PxPhysicsAPI.h>

namespace sapien::ros2 {

class SceneManager;
class SControllableArticulationWrapper;
class JointVelocityController;

class RobotManager {
  friend class SceneManager;

public:
  SControllableArticulationWrapper *mWrapper;

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
  std::vector<std::shared_ptr<JointVelocityController>> mJointVelocityControllers = {};

public:
  RobotManager(SControllableArticulationWrapper *wrapper, const std::string &nameSpace,
               const std::string &robotName, rclcpp::Clock::SharedPtr clock);

  void setDriveProperty(float stiffness, float damping, float forceLimit = PX_MAX_F32,
                        const std::vector<uint32_t> &jointIndex = {});

  void balancePassiveForce(bool gravity = true, bool coriolisAndCentrifugal = true,
                           bool external = true);

  // Create controller and publisher
  void createJointPublisher(double pubFrequency);
  std::weak_ptr<JointVelocityController>
  buildJointVelocityController(const std::vector<std::string> &jointNames,
                               const std::string &serviceName) {
    auto controller = std::make_shared<JointVelocityController>(mNameSpace, mNode, mClock,
                                                                mWrapper, jointNames, serviceName);
    mJointVelocityControllers.push_back(controller);
    return std::weak_ptr<JointVelocityController>(controller);
  };

protected:
  void updateJointStates(const std::vector<float> &jointAngles,
                         const std::vector<float> &jointVelocity);

  // Step function when simulator step once
  void step();

  void start() {}
};
} // namespace sapien::ros2
