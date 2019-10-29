//
// Created by sim on 10/5/19.
//

#include "controller_manger.h"

#include <utility>
namespace sapien::robot {

sapien::robot::ControllerManger::ControllerManger(std::string robotName,
                                                  sapien::ControllableArticulationWrapper *wrapper)
    : wrapper(wrapper), robotName(std::move(robotName)), spinner(4),
      time_step(wrapper->informMangerTimestepChange()) {
  if (!ros::isInitialized()) {
  }
  nh = std::make_unique<ros::NodeHandle>();
  jointName = wrapper->get_drive_joint_name();
}
void sapien::robot::ControllerManger::createJointPubNode(double pubFrequency,
                                                         double updateFrequency) {
  if (jointPubNode) {
    ROS_WARN("Joint Pub Node has already been created for this robot controller manager");
    ROS_WARN("Will use the original joint state pub node");
    return;
  }
  jointPubNode =
      std::make_unique<JointPubNode>(wrapper, pubFrequency, updateFrequency, robotName, nh.get());
  mStates = jointPubNode->mStates.get();
}
CartesianVelocityController *
sapien::robot::ControllerManger::createCartesianVelocityController(const std::string &groupName) {
  if (name2CartesianVelocityController.find(groupName) != name2CartesianVelocityController.end()) {
    ROS_WARN("Cartesian Velocity Controller has already existed for the same group name: %s",
             groupName.c_str());
    return nullptr;
  }
  auto controller = std::make_unique<CartesianVelocityController>(wrapper, groupName, time_step,
                                                                  nh.get(), robotName);

  auto controllerPtr = controller.get();
  name2CartesianVelocityController[groupName] = std::move(controller);
  return controllerPtr;
}
JointVelocityController *sapien::robot::ControllerManger::createJointVelocityController(
    const std::vector<std::string> &jointNames, const std::string &serviceName) {
  assert(jointPubNode);
  for (const auto &name : jointNames) {
    if (std::count(jointName.begin(), jointName.end(), name) != 1) {
      ROS_WARN("Joint name not found in robot %s: %s", robotName.c_str(), name.c_str());
      return nullptr;
    }
  }

  std::unique_ptr<JointVelocityController> controller = std::make_unique<JointVelocityController>(
      wrapper, jointNames, serviceName, time_step, nh.get(), robotName);

  auto controllerPtr = controller.get();
  name2JointVelocityController[serviceName] = std::move(controller);
  return controllerPtr;
}
void sapien::robot::ControllerManger::createGroupTrajectoryController(
    const std::string &groupName) {
  assert(jointPubNode);
  if (name2GroupTrajectoryController.find(groupName) != name2GroupTrajectoryController.end()) {
    ROS_WARN("Cartesian Velocity Controller has already existed for the same group name: %s",
             groupName.c_str());
    return;
  }
  auto controller =
      std::make_unique<GroupControllerNode>(wrapper, groupName, time_step, nh.get(), robotName);
  name2GroupTrajectoryController[groupName] = std::move(controller);
}

ros::NodeHandle *sapien::robot::ControllerManger::getHandle() const { return nh.get(); }
std::string sapien::robot::ControllerManger::getRobotName() const { return robotName; }
void sapien::robot::ControllerManger::start() { spinner.start(); }
void sapien::robot::ControllerManger::stop() { spinner.stop(); }
void sapien::robot::ControllerManger::removeController(const std::string &) {}
void ControllerManger::moveBase(const PxTransform &T) { wrapper->articulation->move_base(T); }
} // namespace sapien::robot
