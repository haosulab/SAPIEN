//
// Created by sim on 10/3/19.
//

#include "cartesian_velocity_controller.h"
#include <sensor_msgs/JointState.h>

#include <memory>
#include <utility>
namespace sapien::robot {

CartesianVelocityController::CartesianVelocityController(ControllableArticulationWrapper *wrapper,
                                                         const std::string &groupName,
                                                         float timestep, ros::NodeHandle *nh,
                                                         const std::string &robotName)
    : mNodeHandle(std::move(nh)), timestep(timestep) {
  // TODO robot description name
  loader = robot_model_loader::RobotModelLoader("robot_description");
  kinematicModel = loader.getModel();
  ROS_INFO("Model frame: %s", kinematicModel->getModelFrame().c_str());

  state = std::make_unique<robot_state::RobotState>(kinematicModel);
  state->setToDefaultValues();
  jointModelGroup = state->getJointModelGroup(groupName);
  jointName = jointModelGroup->getVariableNames();
  jointValue.resize(jointName.size(), 0);
  jointValueFloat.resize(jointName.size(), 0);

  eeName = jointModelGroup->getOnlyOneEndEffectorTip()->getName();
  currentPose = state->getGlobalLinkTransform(eeName);
  jointStateTopicName = "/" + robotName + "/joint_states";

  // Build relative transformation matrix cache
  transStepSize = timestep * 1;
  rotStepSize = timestep * 1;
  buildCartesianAngularVelocityCache();
  buildCartesianVelocityCache();

  // Register queue
  mQueue = std::make_unique<ThreadSafeQueue>();
  wrapper->add_position_controller(jointName, mQueue.get());
}
void CartesianVelocityController::updateCurrentPose() {
  // Check if the joint index has already been cached
  auto jointStatePtr =
      ros::topic::waitForMessage<sensor_msgs::JointState>(jointStateTopicName, ros::Duration(10));
  if (jointIndex2Topic.empty()) {
    std::vector<std::string> topicJointNames = jointStatePtr->name;
    for (auto &i : jointName) {
      auto index = std::find(topicJointNames.begin(), topicJointNames.end(), i);
      if (index == topicJointNames.end()) {
        ROS_ERROR("Joint name in controller not found in joint topic: %s", i.c_str());
      } else {
        jointIndex2Topic.push_back(index - topicJointNames.begin());
      }
    }
  }
  for (size_t i = 0; i < jointValue.size(); i++) {
    jointValue[i] = jointStatePtr->position[jointIndex2Topic[i]];
  }
  state->setJointGroupPositions(jointModelGroup, jointValue);
  currentPose = state->getGlobalLinkTransform(eeName);
}
void CartesianVelocityController::moveRelative(CartesianCommand type, bool continuous) {
  if (!continuous) {
    updateCurrentPose();
    currentPose = state->getGlobalLinkTransform(eeName);
  }
  auto newPose = cartesianMatrix[type] * currentPose;
  bool found_ik = state->setFromIK(jointModelGroup, newPose, 0.01);
  if (!found_ik) {
    ROS_WARN("Ik not found without timeout");
    return;
  }
  if (jointJumpCheck) {
    std::vector<double> currentJointValue;
    state->copyJointGroupPositions(jointModelGroup, currentJointValue);
    if (testJointSpaceJump(currentJointValue, jointValue, 0.2)) {
      ROS_WARN("Joint space jump! Not execute");
      return;
    }
  }
  state->copyJointGroupPositions(jointModelGroup, jointValue);
  currentPose = newPose;

  // Control the physx via controllable wrapper
  jointValueFloat.assign(jointValue.begin(), jointValue.end());
  mQueue->pushValue(jointValueFloat);
}
float CartesianVelocityController::getVelocity() const { return transStepSize / timestep; }
void CartesianVelocityController::setVelocity(float v) {
  transStepSize = v * timestep;
  buildCartesianVelocityCache();
}
float CartesianVelocityController::getAngularVelocity() const { return rotStepSize / timestep; }
void CartesianVelocityController::setAngularVelocity(float v) {
  rotStepSize = timestep * v;
  buildCartesianAngularVelocityCache();
}

void CartesianVelocityController::buildCartesianVelocityCache() {
  // X, Y, Z translation
  cartesianMatrix[0](0, 3) = transStepSize;
  cartesianMatrix[1](1, 3) = transStepSize;
  cartesianMatrix[2](2, 3) = transStepSize;
  cartesianMatrix[6](0, 3) = -transStepSize;
  cartesianMatrix[7](1, 3) = -transStepSize;
  cartesianMatrix[8](2, 3) = -transStepSize;
}
void CartesianVelocityController::buildCartesianAngularVelocityCache() {
  cartesianMatrix[3].matrix() << cos(rotStepSize), sin(rotStepSize), 0, 0, -sin(rotStepSize),
      cos(rotStepSize), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  cartesianMatrix[4].matrix() << cos(rotStepSize), 0, -sin(rotStepSize), 0, 0, 1, 0, 0,
      sin(rotStepSize), 0, cos(rotStepSize), 0, 0, 0, 0, 1;
  cartesianMatrix[5].matrix() << cos(rotStepSize), sin(rotStepSize), 0, 0, -sin(rotStepSize),
      cos(rotStepSize), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  cartesianMatrix[9].matrix() << cos(-rotStepSize), sin(-rotStepSize), 0, 0, -sin(-rotStepSize),
      cos(-rotStepSize), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  cartesianMatrix[10].matrix() << cos(-rotStepSize), 0, -sin(-rotStepSize), 0, 0, 1, 0, 0,
      sin(-rotStepSize), 0, cos(-rotStepSize), 0, 0, 0, 0, 1;
  cartesianMatrix[11].matrix() << cos(-rotStepSize), sin(-rotStepSize), 0, 0, -sin(-rotStepSize),
      cos(-rotStepSize), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
}
void CartesianVelocityController::toggleJumpTest(bool enable) { jointJumpCheck = enable; }
bool CartesianVelocityController::testJointSpaceJump(const std::vector<double> &q1,
                                                     const std::vector<double> &q2,
                                                     double threshold) {
  double distance = 0;
  for (size_t i = 0; i < q1.size(); ++i) {
    distance += abs(q1[i] - q2[i]);
  }
  return distance / q1.size() > threshold;
}
} // namespace sapien::robot