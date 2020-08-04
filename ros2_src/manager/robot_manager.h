#pragma once

#include <numeric>
#include <utility>
#include <vector>

#include "controller/cartesian_velocity_controller.h"
#include "controller/joint_publisher.h"
#include "controller/joint_velocity_controller.h"
#include "controller/motion_planner.h"
#include "manager/kinematics_config.hpp"
#include "manager/motion_planning_config.hpp"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace sapien::ros2 {

class SceneManager;
class SControllableArticulationWrapper;
class RobotLoader;

class RobotManager {
  friend SceneManager;
  friend RobotLoader;

public:
  SControllableArticulationWrapper *mWrapper;
  static std::string SAPIEN_ROS2_RESOURCES_DIRECTORY; // No / in this string

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

  // Options
  MotionPlanningConfig mPlanningConfig;
  KinematicsConfig mKinematicsConfig;
  std::unique_ptr<moveit::planning_interface::MoveItCpp::Options> mMoveitCppOption = nullptr;

  // Controllers
  // TODO: use unique ptr, since controller handle make no sense without robot state
  std::unique_ptr<JointPublisher> mJointPublisher = nullptr;
  std::vector<std::shared_ptr<JointVelocityController>> mJointVelocityControllers = {};
  std::vector<std::shared_ptr<CartesianVelocityController>> mCartesianVelocityControllers = {};
  std::vector<std::unique_ptr<MotionPlanner>> mMotionPlanners = {};

  // Robot model
  robot_model_loader::RobotModelLoaderUniquePtr mRobotLoader = nullptr;
  robot_model::RobotModelPtr mRobotModel = nullptr;
  robot_state::RobotStateUniquePtr mRobotState = nullptr;
  moveit::planning_interface::MoveItCppPtr mMoveItCpp = nullptr;

public:
  RobotManager(SControllableArticulationWrapper *wrapper, const std::string &nameSpace,
               const std::string &robotName, rclcpp::Clock::SharedPtr clock);

  void init();
  static inline std::string getResourcesDirectory() { return SAPIEN_ROS2_RESOURCES_DIRECTORY; }
  static void setResourcesDirectory(const std::string &path);

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
  MotionPlanner *buildMotionPlanner(const std::string &groupName, const std::string &serviceName);

  // Config
  inline KinematicsConfig getKinematicsConfig() const { return mKinematicsConfig; }
  inline MotionPlanningConfig getMotionPlanningConfig() const { return mPlanningConfig; }

  bool setKinematicsConfig(const KinematicsConfig &config = KinematicsConfig());
  bool setMotionPlanningConfig(const MotionPlanningConfig &config = MotionPlanningConfig());

  // Robot Level Control
  inline const std::vector<std::string> &getGroupNames() {
    return mRobotModel->getJointModelGroupNames();
  }

  // Articulation function
  physx::PxTransform getRootPose();

protected:
  void updateStates(const std::vector<float> &jointPosition,
                    const std::vector<float> &jointVelocity);

  // Step function when simulator step once
  void step(bool timeStepChange = false, float newTimeStep = 0);
  void start() {}
};
} // namespace sapien::ros2
