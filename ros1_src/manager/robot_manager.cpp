#include "robot_manager.h"
#include "articulation/sapien_joint.h"
#include "manager/sapien_controllable_articulation.h"
#include "scene_manager.h"
#include <vector>

namespace sapien::ros1 {

RobotManager::RobotManager(SControllableArticulationWrapper *wrapper, const std::string &robotName,
                           const ros::NodeHandlePtr& parentNode, double frenquency)
    : mWrapper(wrapper), mNode(new ros::NodeHandle(*parentNode, robotName)),
      mRobotHW(mNode, wrapper, robotName, frenquency) {
  auto logger = spdlog::get("SAPIEN_ROS1");
}

// void RobotManager::init() {
//  auto logger = spdlog::get("SAPIEN_ROS2");
//  rclcpp::Parameter paramUseSimTime("use_sim_time", true);
//  mNode->set_parameter(paramUseSimTime);
//  // Load robot description from remote node, do not use node to access parameters
//  // SAPIEN convention: the remote node name must be "$/{robotName}_config"
//  // Note that you must add a "/" before the name of the node, otherwise it do not exist
//  const std::string robotURDFName = "robot_description";
//  const std::string robotSRDFName = "robot_description_semantic";
//  const std::string robotConfigNodeName = "/" + std::string(mNode->get_name()) + "_config";
//
//  if (mNode->has_parameter(robotURDFName) && mNode->has_parameter(robotSRDFName)) {
//
//    // Load robot and robot state
//    auto opt = robot_model_loader::RobotModelLoader::Options();
//    opt.load_kinematics_solvers_ = false;
//    mRobotLoader = std::make_unique<robot_model_loader::RobotModelLoader>(mNode, opt);
//    mRobotModel = mRobotLoader->getModel();
//    logger->info("Load ROS robot model {}, base frame: {}", robotURDFName,
//                 mRobotModel->getModelFrame());
//    mRobotState = std::make_unique<robot_state::RobotState>(mRobotModel);
//
//    // Build up index for transformation from simulation to robot state
//    auto variableNames = mRobotState->getVariableNames();
//    std::vector<std::string> jointName = mJointStates->name;
//    if (variableNames.size() != mJointNum) {
//      logger->error("Robot State has different dof from robot articulation");
//      exit(0);
//    }
//    for (auto &variableName : variableNames) {
//      auto iter = std::find(jointName.begin(), jointName.end(), variableName);
//      if (iter == jointName.end()) {
//        logger->error("Robot State variable name {} not found in articulation joint names",
//                      variableName);
//        exit(0);
//      }
//      uint32_t index = iter - jointName.begin();
//      mJointIndex.push_back(index);
//    }
//    mLoadRobot = true;
//  } else {
//    logger->warn("No parameter %s found for ROS2, building robot manager fail");
//    logger->warn("Inverse kinematics and motion planning features will be disabled!");
//    mLoadRobot = false;
//  }
//  setKinematicsConfig();
//  setMotionPlanningConfig();
//}

void RobotManager::setDriveProperty(float stiffness, float damping, float forceLimit,
                                    const std::vector<uint32_t> &jointIndex) {
  std::vector<uint32_t> index(jointIndex);
  if (jointIndex.empty()) {
    index.resize(mWrapper->mJoints.size());
    std::iota(std::begin(index), std::end(index), 0);
  }

  for (unsigned int i : index) {
    assert(i < mWrapper->mArticulation->dof());
    auto joint = mWrapper->mJoints[i];
    joint->setDriveProperty(stiffness, damping, forceLimit);
  }
}

void RobotManager::balancePassiveForce(bool gravity, bool coriolisAndCentrifugal, bool external) {
  auto balanceForce =
      mWrapper->mArticulation->computePassiveForce(gravity, coriolisAndCentrifugal, external);
  mWrapper->mArticulation->setQf(balanceForce);
}
} // namespace sapien::ros1
