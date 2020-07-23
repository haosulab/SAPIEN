#pragma once

#include <numeric>
#include <utility>
#include <vector>

#include "manager/kinematics_config.hpp"
#include "manager/motion_planning_config.hpp"

#include <foundation/PxSimpleTypes.h>
//#include <moveit/robot_model/robot_model.h>
//#include <moveit/robot_model_loader/robot_model_loader.h>
#include <manager/robot_hw.hpp>
#include <ros/ros.h>

namespace sapien::ros1 {

class SceneManager;
class SControllableArticulationWrapper;
class SRobotHW;
// class RobotLoader;

class RobotManager {
  friend SceneManager;
  //  friend RobotLoader;

protected:
  // Basic handle
  SControllableArticulationWrapper *mWrapper;
  ros::NodeHandlePtr mNode;
  SRobotHW mRobotHW;
  SceneManager *mSceneManager;

  // Cache
  std::vector<uint32_t> mJointIndex;
  uint32_t mJointNum = 0;

  // Robot model
  //  robot_model_loader::RobotModelLoaderUniquePtr mRobotLoader = nullptr;
  //  robot_model::RobotModelPtr mRobotModel = nullptr;
  //  robot_state::RobotStateUniquePtr mRobotState = nullptr;
  //  moveit::planning_interface::MoveItCppPtr mMoveItCpp = nullptr;

public:
  RobotManager(SControllableArticulationWrapper *wrapper, const std::string &robotName,
               const ros::NodeHandlePtr& parentNode, double freqency);

  void init();

  void setDriveProperty(float stiffness, float damping, float forceLimit = PX_MAX_F32,
                        const std::vector<uint32_t> &jointIndex = {});

  void balancePassiveForce(bool gravity = true, bool coriolisAndCentrifugal = true,
                           bool external = true);

  // Create controller and publisher
  //  void createJointPublisher(double pubFrequency);
  //  inline KinematicsConfig getKinematicsConfig() const { return mKinematicsConfig; }
  //  inline MotionPlanningConfig getMotionPlanningConfig() const { return mPlanningConfig; }

  //  bool setKinematicsConfig(const KinematicsConfig &config = KinematicsConfig());
  //  bool setMotionPlanningConfig(const MotionPlanningConfig &config = MotionPlanningConfig());

  // Robot Level Control
  //  inline const std::vector<std::string> &getGroupNames() {
  //    return mRobotModel->getJointModelGroupNames();
  //  }

protected:
  //  void updateStates(const std::vector<float> &jointPosition,
  //                    const std::vector<float> &jointVelocity);

  // Step function when simulator step once
  void step(bool timeStepChange = false, float newTimeStep = 0){};

  void start() {}
};
} // namespace sapien::ros1
