#include "joint_velocity_controller.h"
#include "sapien_controllable_articulation.h"
#include <utility>
#include <vector>
namespace sapien::ros2 {

sapien::ros2::JointVelocityController::JointVelocityController(
    rclcpp::Node::SharedPtr node, rclcpp::Clock::SharedPtr clock,
    sapien::ros2::SControllableArticulationWrapper *wrapper,
    const std::vector<std::string> &jointNames, const std::string &serviceName, double latency)
    : DelayedControllerBase(std::move(clock), latency, 2), mNode(std::move(node)),
      mJointNames(jointNames), mContinuousCommands(jointNames.size()), mCommands() {
  // Create Service
  mService = mNode->create_service<sapien_ros2_communication_interface::srv::JointVelocity>(
      std::string(mNode->get_name()) + "/" + serviceName,
      [this](const std::shared_ptr<sapien_ros2_communication_interface::srv::JointVelocity_Request>
                 req,
             std::shared_ptr<sapien_ros2_communication_interface::srv::JointVelocity_Response> res)
          -> void { this->handleService(req, std::move(res)); });

  // Register command
  wrapper->registerContinuousVelocityCommands(&mContinuousCommands, mJointNames,
                                              &(mCommandTimer[1]));
  wrapper->registerVelocityCommands(&mCommands, mJointNames, &(mCommandTimer[0]));
}
void sapien::ros2::JointVelocityController::moveJoint(const std::vector<float> &velocity,
                                                      bool continuous) {
  if (velocity.size() != mJointNames.size()) {
    spdlog::error("Velocity vector size %i does not match joint number %i", velocity.size(),
                  mJointNames.size());
    return;
  }
  HANDLE_COMMAND(continuous, velocity)
}
void sapien::ros2::JointVelocityController::moveJoint(const std::vector<std::string> &jointNames,
                                                      const std::vector<float> &velocity,
                                                      bool continuous) {
  std::vector<float> command(mJointNames.size(), 0);
  for (size_t i = 0; i < mJointNames.size(); ++i) {
    auto iter = std::find(mJointNames.begin(), mJointNames.end(), jointNames[i]);
    if (iter == mJointNames.end()) {
      spdlog::error("Can not find joint name %s in controller joint names",
                    mJointNames[i].c_str());
      return;
    }
    auto index = iter - mJointNames.begin();
    command[index] = velocity[i];
  }
  HANDLE_COMMAND(continuous, command)
}
void sapien::ros2::JointVelocityController::moveJoint(const std::vector<std::string> &jointNames,
                                                      float velocity, bool continuous) {
  std::vector<float> command(mJointNames.size(), 0);
  for (size_t i = 0; i < mJointNames.size(); ++i) {
    auto iter = std::find(mJointNames.begin(), mJointNames.end(), jointNames[i]);
    if (iter == mJointNames.end()) {
      spdlog::error("Can not find joint name %s in controller joint names",
                    mJointNames[i].c_str());
      return;
    }
    auto index = iter - mJointNames.begin();
    command[index] = velocity;
  }
  HANDLE_COMMAND(continuous, command)
}
void sapien::ros2::JointVelocityController::handleService(
    const std::shared_ptr<sapien_ros2_communication_interface::srv::JointVelocity_Request> req,
    std::shared_ptr<sapien_ros2_communication_interface::srv::JointVelocity_Response> res) {
  std::vector<float> jointVelocity = req->velocity;
  std::vector<std::string> jointName = req->name;
  if (req->name.empty()) {
    jointName = mJointNames;
  }

  // Joint name should have same size as joint velocity
  if (jointVelocity.size() != mJointNames.size()) {
    RCLCPP_WARN(mNode->get_logger(),
                "Joint Velocity Server receive service request with %i velocity, not same as "
                "joint numbers %i",
                jointVelocity.size(), mJointNames.size());
    res->set__success(false);
    return;
  }

  // Parse order of joint name and joint velocity
  std::vector<float> velocityCommands(mJointNames.size(), 0);
  for (size_t i = 0; i < jointName.size(); ++i) {
    auto index =
        std::find(mJointNames.begin(), mJointNames.end(), jointName[i]) - mJointNames.begin();
    velocityCommands[index] = jointVelocity[i];
  }

  if (req->continuous) {
    mContinuousCommands.write(velocityCommands);
    updateCommandTimer(req->stamp, 1);
  } else {
    mCommands.push(velocityCommands);
    updateCommandTimer(req->stamp, 0);
  }
  res->set__success(true);
}
} // namespace sapien::ros2
