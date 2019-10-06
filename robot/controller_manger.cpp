//
// Created by sim on 10/5/19.
//

#include "controller_manger.h"
sapien::robot::ControllerManger::ControllerManger(const std::string &robotName,
                                                  sapien::ControllableArticulationWrapper *wrapper)
    : wrapper(wrapper), robotName(robotName), timestep(wrapper->informMangerTimestepChange()),
      spinner(4) {
  nh = std::make_unique<ros::NodeHandle>();
}
void sapien::robot::ControllerManger::createJointPubNode(double pubFrequency,
                                                         double updateFrequency) {
  if (jointPubNode) {
    ROS_WARN("Joint Pub Node has already been created for this robot controller manager");
    ROS_WARN("Will use the original joint state pub node");
  }
  jointPubNode =
      std::make_unique<JointPubNode>(wrapper, pubFrequency, updateFrequency, robotName, nh.get());
}
void sapien::robot::ControllerManger::createCartesianVelocityController(
    const std::string &groupName) {
  if (name2CartesianVelocityController.find(groupName) != name2CartesianVelocityController.end()) {
    ROS_WARN("Cartesian Velocity Controller has already existed for the same group name: %s",
             groupName.c_str());
    return;
  }
  auto controller = std::make_unique<CartesianVelocityController>(wrapper, groupName, timestep,
                                                                  nh.get(), robotName);
  name2CartesianVelocityController[groupName] = std::move(controller);
}
void sapien::robot::ControllerManger::createJointVelocityController(
    const std::vector<std::string> &jointNames, const std::string &serviceName) {
  std::unique_ptr<VelocityControllerServer> controller =
      std::make_unique<VelocityControllerServer>(wrapper, jointNames, serviceName, timestep,
                                                 nh.get(), robotName);

  name2JointVelocityController[serviceName] = std::move(controller);
}
void sapien::robot::ControllerManger::createGroupTrajectoryController(
    const std::string &groupName) {
  if (name2GroupTrajectoryController.find(groupName) != name2GroupTrajectoryController.end()) {
    ROS_WARN("Cartesian Velocity Controller has already existed for the same group name: %s",
             groupName.c_str());
    return;
  }
  auto controller =
      std::make_unique<GroupControllerNode>(wrapper, groupName, timestep, nh.get(), robotName);
  name2GroupTrajectoryController[groupName] = std::move(controller);
}

ros::NodeHandle *sapien::robot::ControllerManger::getHandle() const { return nh.get(); }
std::string sapien::robot::ControllerManger::getRobotName() const { return robotName; }
void sapien::robot::ControllerManger::start() {
  spinner.start(); }
void sapien::robot::ControllerManger::stop() { spinner.stop(); }
void sapien::robot::ControllerManger::removeController(const std::string &) {}
