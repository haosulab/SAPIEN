#pragma once

#include <numeric>
#include <utility>

#include "controller/cartesian_velocity_controller.h"
#include "controller/joint_publisher.h"
#include "controller/joint_velocity_controller.h"
#include "controller/motion_planner.h"
#include "manager/kinematics_config.hpp"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>

namespace sapien::ros2 {

static std::string SAPIEN_ROS2_RESOURCES_DIRECTORY; // No / in this string

inline void setResourcesDirectory(const std::string &path) {
  SAPIEN_ROS2_RESOURCES_DIRECTORY = fs::absolute(path);
}

class SceneManager;
class SControllableArticulationWrapper;
// class JointVelocityController;
class RobotLoader;
// class MotionPlanner;

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

  MotionPlanner *buildMotionPlanner(const std::string &groupName, const std::string &serviceName) {
    auto logger = spdlog::get("SAPIEN_ROS2");
    if (!mLoadRobot) {
      logger->error("No robot load from parameter server, fail to build motion planner!");
      assert(mLoadRobot);
    }
    if (!mMoveItCpp)
      mMoveItCpp.reset(new MoveItCpp(mNode, *mMoveitCppOption));
    auto planner =
        std::make_unique<MotionPlanner>(mNode, mClock, mMoveItCpp, groupName, serviceName);
    auto plannerPtr = planner.get();
    mMotionPlanners.push_back(std::move(planner));
    return plannerPtr;
  };

  // Robot Level Control
  inline const std::vector<std::string> &getGroupNames() {
    return mRobotModel->getJointModelGroupNames();
  }

  inline KinematicsConfig getKinematicsConfig() const { return mKinematicsConfig; }
  inline MotionPlanningConfig getMotionPlanningConfig() const { return mPlanningConfig; }

  bool setKinematicsConfig(const KinematicsConfig &config = KinematicsConfig()) {
    for (const auto &g : mRobotModel->getJointModelGroups()) {
      if (g->isChain())
        config.publishKinematicsConfig(mNode, g->getName());
    }
    mKinematicsConfig = config;
    return true;
  }

  bool setMotionPlanningConfig(const MotionPlanningConfig &config = MotionPlanningConfig()) {
    auto logger = spdlog::get("SAPIEN_ROS2");
    if (!mMoveItCpp) {
      logger->info("Init Motion Planning Config");
    }
    if (!mMotionPlanners.empty()) {
      logger->error("Motion Planning Config can not be set after creating motion planner!");
    }

    // Load default parameters for scene monitor
    //   TODO: follow moveit2 update to modify monitorOptions.wait_for_initial_state_timeout
    moveit::planning_interface::MoveItCpp::PlanningSceneMonitorOptions monitorOptions;
    monitorOptions.load(mNode);
    monitorOptions.joint_state_topic = mNameSpace + "/joint_states";
    monitorOptions.attached_collision_object_topic = mNameSpace + "/planning_scene_monitor";
    monitorOptions.monitored_planning_scene_topic = mNameSpace + "/monitored_planning_scene";
    monitorOptions.publish_planning_scene_topic = mNameSpace + "/publish_planning_scene";

    // Load default parameters for planning pipeline
    moveit::planning_interface::MoveItCpp::PlanningPipelineOptions pipelineOptions;
    pipelineOptions.parent_namespace = mNameSpace;
    pipelineOptions.pipeline_names = {"ompl"}; // Current only support ompl

    // Load Parameter container for initializing MoveItCpp
    moveit::planning_interface::MoveItCpp::Options options(mNode);
    options.planning_pipeline_options = pipelineOptions;
    options.planning_scene_monitor_options = monitorOptions;

    // Load parameters from motion planning config
    config.publishPlannerOption(mNode);
    config.publishGeneralOMPLConfig(mNode);
    auto parseFile =
        config.publishDefaultOMPLPlannerConfig(mNode, SAPIEN_ROS2_RESOURCES_DIRECTORY);

    mPlanningConfig = config;
    *mMoveitCppOption = options;
    return parseFile;
  };

protected:
  void updateStates(const std::vector<float> &jointPosition,
                    const std::vector<float> &jointVelocity);

  // Step function when simulator step once
  void step(bool timeStepChange = false, float newTimeStep = 0);

  void start() {}
};
} // namespace sapien::ros2
