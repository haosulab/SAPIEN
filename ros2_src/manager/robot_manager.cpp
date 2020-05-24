#include "robot_manager.h"
#include "articulation/sapien_joint.h"
#include "controller/sapien_controllable_articulation.h"
#include "scene_manager.h"
#include <vector>

namespace sapien::ros2 {

RobotManager::RobotManager(SControllableArticulationWrapper *wrapper, const std::string &nameSpace,
                           const std::string &robotName, rclcpp::Clock::SharedPtr clock)
    : mWrapper(wrapper), mClock(std::move(clock)), mNameSpace(nameSpace + "/" + robotName) {

  mNode = rclcpp::Node::make_shared(robotName, nameSpace);

  // Create Initial Joint States
  mJointStates = std::make_unique<sensor_msgs::msg::JointState>();
  auto jointName = wrapper->getDriveJointNames();
  mJointNum = jointName.size();
  mJointStates->name = jointName;
  mJointStates->position.resize(jointName.size());
  mJointStates->velocity.resize(jointName.size());
}

void RobotManager::init() {
  auto logger = spdlog::get("SAPIEN_ROS2");
  // Load robot description from remote node, do not use node to access parameters
  // SAPIEN convention: the remote node name must be "$/{robotName}_config"
  // Note that you must add a "/" before the name of the node, otherwise it do not exist
  const std::string robotURDFName = "robot_description";
  const std::string robotSRDFName = "robot_description_semantic";
  const std::string robotConfigNodeName = "/" + std::string(mNode->get_name()) + "_config";

  //  rclcpp::SyncParametersClient paramClient(mNode.get(), robotConfigNodeName);
  //  while (!paramClient.wait_for_service(1s)) {
  //    if (!rclcpp::ok()) {
  //      RCLCPP_ERROR(mNode->get_logger(), "Interrupted while waiting for the service. Exiting.");
  //      rclcpp::shutdown();
  //    }
  //    RCLCPP_INFO(mNode->get_logger(), "service not available, waiting again...");
  //  }

  //  if (paramClient.has_parameter(robotURDFName) && paramClient.has_parameter(robotSRDFName)) {
  //    // Remapping config parameter to current node and load robot model
  //    // With current convention, a specific robot will have one kinematics parameters
  //    // No matter how many time it is instantiated
  //    std::vector<std::string> key = paramClient.list_parameters({}, 10).names;
  //    std::vector<rclcpp::Parameter> value = paramClient.get_parameters(key);
  //    for (auto &j : value) {
  //      if (mNode->has_parameter(j.get_name()))
  //        continue;
  //      mNode->declare_parameter(j.get_name(), j.get_parameter_value());
  //    }
  //  }
  if (mNode->has_parameter(robotURDFName) && mNode->has_parameter(robotSRDFName)) {

    // Load robot and robot state
    mRobotLoader = std::make_unique<robot_model_loader::RobotModelLoader>(mNode);
    mRobotModel = mRobotLoader->getModel();
    logger->info("Load ROS robot model {}, base frame: {}", robotURDFName,
                 mRobotModel->getModelFrame());
    mRobotState = std::make_unique<robot_state::RobotState>(mRobotModel);

    // Build up index for transformation from simulation to robot state
    auto variableNames = mRobotState->getVariableNames();
    std::vector<std::string> jointName = mJointStates->name;
    if (variableNames.size() != mJointNum) {
      logger->error("Robot State has different dof from robot articulation");
      exit(0);
    }
    for (auto &variableName : variableNames) {
      auto iter = std::find(jointName.begin(), jointName.end(), variableName);
      if (iter == jointName.end()) {
        logger->error("Robot State variable name {} not found in articulation joint names",
                      variableName);
        exit(0);
      }
      uint32_t index = iter - jointName.begin();
      mJointIndex.push_back(index);
    }
    mLoadRobot = true;
  } else {
    logger->warn("No parameter %s found for ROS2, building robot manager fail");
    logger->warn("Inverse kinematics and motion planning features will be disabled!");
    mLoadRobot = false;
  }
}

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
void RobotManager::updateStates(const std::vector<float> &jointPosition,
                                const std::vector<float> &jointVelocity) {
  // Update joint state message
  mJointStates->position.assign(jointPosition.begin(), jointPosition.begin() + mJointNum);
  mJointStates->velocity.assign(jointVelocity.begin(), jointVelocity.begin() + mJointNum);
  mJointStates->header.stamp = mClock->now();

  // Update robot state
  std::vector<double> robotStateVariable(mJointNum);
  for (size_t i = 0; i < mJointNum; ++i) {
    robotStateVariable[i] = double(jointPosition[mJointIndex[i]]);
  }
  mRobotState->setVariablePositions(robotStateVariable);
  mRobotState->update();
}
void RobotManager::step(bool timeStepChange, float newTimeStep) {
  updateStates(mWrapper->mJointPositions.read(), mWrapper->mJointVelocities.read());
  for (auto &mCartesianVelocityController : mCartesianVelocityControllers) {
    if (timeStepChange) {
      mCartesianVelocityController->mTimeStep = double(newTimeStep);
    }
    if (mCartesianVelocityController->mCurrentStepCalled) {
      mCartesianVelocityController->mLastStepCalled = true;
      mCartesianVelocityController->mCurrentStepCalled = false;
    } else if (!mCartesianVelocityController->mCurrentStepCalled &&
               mCartesianVelocityController->mLastStepCalled) {
      mCartesianVelocityController->mLastStepCalled = false;
    }
  }
}

// Create controller and publisher
void RobotManager::createJointPublisher(double pubFrequency) {
  auto logger = spdlog::get("SAPIEN_ROS2");
  if (mJointPublisher) {
    logger->warn("Joint Pub Node has already been created for this Robot Manager");
    logger->warn("Robot Manager will use the original joint state pub node");
    return;
  }

  mJointPublisher = std::make_unique<JointPublisher>(mNameSpace, mNode->shared_from_this(), mClock,
                                                     mJointStates.get(), pubFrequency);
}

JointVelocityController *
RobotManager::buildJointVelocityController(const std::vector<std::string> &jointNames,
                                           const std::string &serviceName, double latency) {
  auto controller = std::make_shared<JointVelocityController>(mNode, mClock, mWrapper, jointNames,
                                                              serviceName, latency);
  mJointVelocityControllers.push_back(controller);
  return controller.get();
}

CartesianVelocityController *
RobotManager::buildCartesianVelocityController(const std::string &groupName,
                                               const std::string &serviceName, double latency) {
  auto logger = spdlog::get("SAPIEN_ROS2");
  if (!mLoadRobot) {
    logger->error("No robot load from parameter server, fail to build cartesian controller!");
    assert(mLoadRobot);
  }
  auto controller = std::make_shared<CartesianVelocityController>(
      mNode, mClock, mWrapper, mRobotState.get(), groupName, serviceName, latency);
  controller->mTimeStep = mSceneManager->mTimeStep;
  mCartesianVelocityControllers.push_back(controller);
  return controller.get();
}
} // namespace sapien::ros2
