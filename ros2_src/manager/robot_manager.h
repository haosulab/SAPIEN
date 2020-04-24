#pragma once

#include <numeric>
#include <utility>

#include "controller/cartesian_velocity_controller.h"
#include "controller/joint_publisher.h"
#include "controller/joint_velocity_controller.h"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>

namespace sapien::ros2 {

class SceneManager;
class SControllableArticulationWrapper;
class JointVelocityController;
class RobotLoader;

class RobotManager {
  friend SceneManager;
  friend RobotLoader;

public:
  SControllableArticulationWrapper *mWrapper;

protected:
  // Basic handle
  rclcpp::Node::SharedPtr mNode;
  sensor_msgs::msg::JointState::UniquePtr mJointStates = nullptr;
  rclcpp::Clock::SharedPtr mClock;
  SceneManager *mSceneManager = nullptr;

  // Cache
  std::string mNameSpace;
  std::vector<uint32_t> mJointIndex;
  uint32_t mJointNum = 0;
  bool mLoadRobot = false;

  // Controllers
  // TODO: use unique ptr, since controller handle make no sense without robot state
  std::unique_ptr<JointPublisher> mJointPublisher = nullptr;
  std::vector<std::shared_ptr<JointVelocityController>> mJointVelocityControllers = {};
  std::vector<std::shared_ptr<CartesianVelocityController>> mCartesianVelocityControllers = {};

  // Robot model
  robot_model_loader::RobotModelLoaderUniquePtr mRobotLoader = nullptr;
  robot_model::RobotModelPtr mRobotModel = nullptr;
  robot_state::RobotStateUniquePtr mRobotState = nullptr;

public:
  RobotManager(SControllableArticulationWrapper *wrapper, const std::string &nameSpace,
               const std::string &robotName, rclcpp::Clock::SharedPtr clock);

  void init();

  void setDriveProperty(float stiffness, float damping, float forceLimit = PX_MAX_F32,
                        const std::vector<uint32_t> &jointIndex = {});

  void balancePassiveForce(bool gravity = true, bool coriolisAndCentrifugal = true,
                           bool external = true);

  // Create controller and publisher
  void createJointPublisher(double pubFrequency);
  JointVelocityController *buildJointVelocityController(const std::vector<std::string> &jointNames,
                                                        const std::string &serviceName,
                                                        double latency = 0);
  CartesianVelocityController *buildCartesianVelocityController(const std::string &groupName,
                                                                const std::string &serviceName,
                                                                double latency = 0);

protected:
  void updateStates(const std::vector<float> &jointPosition,
                    const std::vector<float> &jointVelocity);

  // Step function when simulator step once
  void step(bool timeStepChange = false, float newTimeStep = 0);

  void start() {}
};
} // namespace sapien::ros2
