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
    throw std::runtime_error("ROS not init");
  }
  nh = std::make_unique<ros::NodeHandle>();
  jointName = wrapper->get_drive_joint_name();

  // Create robot states and load robot models
  if (nh->hasParam("robot_description")) {
    loader = std::make_unique<robot_model_loader::RobotModelLoader>("robot_description");
    kinematicModel = loader->getModel();
    ROS_INFO("Model base frame: %s", kinematicModel->getModelFrame().c_str());
    robotState = std::make_unique<robot_state::RobotState>(kinematicModel);
    robotState->setToDefaultValues();
  } else {
    std::cerr << "No robot description found! \n"
              << "Inverse kinematics and motion planning  will be disabled. \n";
  }
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
  jointState = jointPubNode->mStates.get();
}
CartesianVelocityController *
sapien::robot::ControllerManger::createCartesianVelocityController(const std::string &groupName) {
  if (name2CartesianVelocityController.find(groupName) != name2CartesianVelocityController.end()) {
    ROS_WARN("Cartesian Velocity Controller has already existed for the same group name: %s",
             groupName.c_str());
    return nullptr;
  }
  if (!kinematicModel) {
    std::cerr << "Robot has not be loaded, does parameter exist?" << std::endl;
    std::cerr << "Creating cartesian velocity controller fail due to empty robot model"
              << std::endl;
    return nullptr;
  }
  if (!jointPubNode) {
    ROS_WARN("Joint Pub Node has not been created for this robot controller manager");
    ROS_WARN("Try create it first before create any controller");
    return nullptr;
  }
  auto controller = std::make_unique<CartesianVelocityController>(
      wrapper, jointState, robotState.get(), groupName, time_step, nh.get(), robotName);

  auto controllerPtr = controller.get();
  name2CartesianVelocityController[groupName] = std::move(controller);
  return controllerPtr;
}
JointVelocityController *sapien::robot::ControllerManger::createJointVelocityController(
    const std::vector<std::string> &jointNames, const std::string &serviceName) {
  if (!jointPubNode) {
    ROS_WARN("Joint Pub Node has not been created for this robot controller manager");
    ROS_WARN("Try create it first before create any controller");
    return nullptr;
  }
  for (const auto &name : jointNames) {
    if (std::count(jointName.begin(), jointName.end(), name) != 1) {
      ROS_WARN("Joint name not found in robot %s: %s", robotName.c_str(), name.c_str());
      return nullptr;
    }
  }

  std::unique_ptr<JointVelocityController> controller = std::make_unique<JointVelocityController>(
      wrapper, jointNames, jointState, serviceName, time_step, nh.get(), robotName);

  auto controllerPtr = controller.get();
  name2JointVelocityController[serviceName] = std::move(controller);
  return controllerPtr;
}
void sapien::robot::ControllerManger::addGroupTrajectoryController(const std::string &groupName) {
  if (!jointPubNode) {
    ROS_WARN("Joint Pub Node has not been created for this robot controller manager");
    ROS_WARN("Try create it first before create any controller");
    return;
  }
  if (!kinematicModel) {
    std::cerr << "Robot has not be loaded, does parameter exist?" << std::endl;
    std::cerr << "Creating cartesian velocity controller fail due to empty robot model"
              << std::endl;
    return;
  }
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
void sapien::robot::ControllerManger::start() {
  if (spinner.canStart()) {
    spinner.start();
  };
}
void sapien::robot::ControllerManger::stop() { spinner.stop(); }
void sapien::robot::ControllerManger::removeController(const std::string &) {}
void ControllerManger::moveBase(const PxTransform &T) { wrapper->articulation->move_base(T); }
MoveGroupPlanner *ControllerManger::createGroupPlanner(const std::string &groupName) {
  if (!kinematicModel) {
    std::cerr << "Robot has not be loaded, does parameter exist?" << std::endl;
    std::cerr << "Creating cartesian velocity controller fail due to empty robot model"
              << std::endl;
    throw std::runtime_error("Try to create planner before robot model is loaded");
  }
  if (name2MoveGroupPlanner.find(groupName) != name2MoveGroupPlanner.end()) {
    ROS_WARN("Move Group Planner has already existed for the same group name: %s",
             groupName.c_str());
    return nullptr;
  }
  if (name2GroupTrajectoryController.find(groupName) == name2GroupTrajectoryController.end()) {
    ROS_WARN(
        "Move Group Planner require a controller with the same name. Will create one this time.");
    addGroupTrajectoryController(groupName);
  }
  auto planner = std::make_unique<MoveGroupPlanner>(
      groupName, name2GroupTrajectoryController[groupName].get(), nh.get());
  auto plannerPtr = planner.get();
  name2MoveGroupPlanner[groupName] = std::move(planner);
  return plannerPtr;
}
} // namespace sapien::robot
