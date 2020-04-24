#include "cartesian_velocity_controller.h"

#include "sapien_controllable_articulation.h"
#include <utility>

namespace sapien::ros2 {

sapien::ros2::CartesianVelocityController::CartesianVelocityController(
    rclcpp::Node::SharedPtr node, rclcpp::Clock::SharedPtr clock,
    SControllableArticulationWrapper *wrapper, robot_state::RobotState *robotState,
    const std::string &groupName, const std::string &serviceName, double latency)
    : DelayedControllerBase(std::move(clock), latency), mNode(std::move(node)),
      mRobotState(robotState), mLocalRobotState(*robotState), mVelocityCommand() {
  // Time step will be initialized when build it using robot manager
  // Do not worry if not initialized here
  mJointModelGroup = mLocalRobotState.getJointModelGroup(groupName);
  mJointNames = mJointModelGroup->getActiveJointModelNames();
  mEEName = mJointModelGroup->getLinkModelNames().back();
  mBaseName = mJointModelGroup->getLinkModelNames()[0];
  RCLCPP_INFO(mNode->get_logger(), "Initialize cartesian velocity controller. EE: %s, Base: %s",
              mEEName.c_str(), mBaseName.c_str());

  // Create ROS service
  auto serverName = std::string(mNode->get_name()) + "/" + serviceName;
  mService = mNode->create_service<sapien_ros2_communication_interface::srv::CartesianVelocity>(
      serverName,
      [this](const std::shared_ptr<
                 sapien_ros2_communication_interface::srv::CartesianVelocity_Request>
                 req,
             std::shared_ptr<sapien_ros2_communication_interface::srv::CartesianVelocity_Response>
                 res) -> void { this->handleService(req, std::move(res)); });

  // Register
  wrapper->registerVelocityCommands(&mVelocityCommand, mJointNames, &(mCommandTimer[0]));
}
void sapien::ros2::CartesianVelocityController::moveCartesian(const std::array<double, 3> &vec,
                                                              sapien::ros2::MoveType type) {
  auto logger = spdlog::get("SAPIEN_ROS2");
  if (!mLastStepCalled) {
    synchronizeRealRobotState();
  }
  bool foundIK = false;
  Eigen::VectorXd twist(6);
  Eigen::VectorXd jointVelocity;
  switch (type) {
  case SpatialTwist: {
    logger->warn("Move Cartesian method do you support direct twist type, please use Move Twist");
    return;
  }
  case BodyTwist: {
    logger->warn("Move Cartesian method do you support direct twist type, please use Move Twist");
    return;
  }
  case WorldTranslate: {
    twist << vec[0], vec[1], vec[2], 0, 0, 0;
    twist = transformTwist(twist, mBaseName);
    COMPUTE_VELOCITY_AND_INTEGRATE(mEEName)
    break;
  }
  case WorldRotate: {
    twist << 0, 0, 0, vec[0], vec[1], vec[2];
    twist = transformTwist(twist, mBaseName);
    COMPUTE_VELOCITY_AND_INTEGRATE(mEEName)
    break;
  }
  case LocalTranslate: {
    twist << vec[0], vec[1], vec[2], 0, 0, 0;
    COMPUTE_VELOCITY_AND_INTEGRATE(mEEName)
    break;
  }
  case LocalRotate: {
    twist << 0, 0, 0, vec[0], vec[1], vec[2];
    COMPUTE_VELOCITY_AND_INTEGRATE(mEEName)
    break;
  }
  }
  if (!foundIK) {
    logger->warn("IK not found");
    return;
  }
  std::vector<float> stepVelocity(jointVelocity.data(),
                                  jointVelocity.data() + mJointModelGroup->getVariableCount());
  mVelocityCommand.push(stepVelocity);
  updateCommandTimer(mClock->now());

  // Update control bool
  mCurrentStepCalled = true;
}

void CartesianVelocityController::moveTwist(const std::array<double, 6> &vec, MoveType type) {
  auto logger = spdlog::get("SAPIEN_ROS2");
  if (!mLastStepCalled) {
    synchronizeRealRobotState();
  }
  bool foundIK = false;
  Eigen::VectorXd twist(6);
  twist << vec[0], vec[1], vec[2], vec[3], vec[4], vec[5];
  Eigen::VectorXd jointVelocity;
  if (type < 4) {
    logger->warn("Only twist type is support for move twist function");
    return;
  } else if (type == 4) {
    COMPUTE_VELOCITY_AND_INTEGRATE(mEEName);
  } else {
    twist = transformTwist(twist, mBaseName);
    COMPUTE_VELOCITY_AND_INTEGRATE(mEEName)
  }

  if (!foundIK) {
    logger->warn("IK not found");
    return;
  }
  std::vector<float> stepVelocity(jointVelocity.data(),
                                  jointVelocity.data() + mJointModelGroup->getVariableCount());
  mVelocityCommand.push(stepVelocity);
  updateCommandTimer(mClock->now());

  // Update control bool
  mCurrentStepCalled = true;
}
void sapien::ros2::CartesianVelocityController::synchronizeRealRobotState() {
  // Copy global real time robot state to local robot state
  // TODO: do something when simulation timestep change
  mRobotState->copyJointGroupPositions(mJointModelGroup, mJointValues);
  mLocalRobotState.setJointGroupPositions(mJointModelGroup, mJointValues);
}

void sapien::ros2::CartesianVelocityController::handleService(
    const std::shared_ptr<sapien_ros2_communication_interface::srv::CartesianVelocity_Request> req,
    std::shared_ptr<sapien_ros2_communication_interface::srv::CartesianVelocity_Response> res) {
  if (!mLastStepCalled) {
    synchronizeRealRobotState();
  }

  auto msg = req->twist;
  Eigen::VectorXd twist(6);
  twist << msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z;
  twist = transformTwist(twist, req->frame_id);
  int numStep = std::floor(req->duration.sec / mTimeStep);
  float residual = static_cast<float>(req->duration.sec) - numStep * mTimeStep;

  // Calculate step velocity
  Eigen::VectorXd jointVelocity;
  bool foundIK = true;
  mRobotState->computeVariableVelocity(mJointModelGroup, jointVelocity, twist,
                                       mJointModelGroup->getLinkModel(mEEName));
  std::vector<float> stepVelocity(jointVelocity.data(),
                                  jointVelocity.data() + mJointModelGroup->getVariableCount());
  for (int i = 0; i < numStep; ++i) {
    mVelocityCommand.push(stepVelocity);
    foundIK = mRobotState->integrateVariableVelocity(mJointModelGroup, jointVelocity, mTimeStep) &
              foundIK;
  }

  // Calculate residual velocity
  twist = twist * (residual / mTimeStep);
  mRobotState->computeVariableVelocity(mJointModelGroup, jointVelocity, twist,
                                       mJointModelGroup->getLinkModel(mEEName));
  foundIK =
      mRobotState->integrateVariableVelocity(mJointModelGroup, jointVelocity, mTimeStep) & foundIK;

  // Log
  if (!foundIK)
    RCLCPP_WARN(mNode->get_logger(), "Cartesian velocity service do not find a solution");
  else {
    mVelocityCommand.push(stepVelocity);
    updateCommandTimer(req->time_stamp);
  }

  res->set__success(foundIK);
  // Update control bool
  mCurrentStepCalled = true;
}

Eigen::VectorXd
sapien::ros2::CartesianVelocityController::transformTwist(const Eigen::VectorXd &twist,
                                                          const std::string &frame) {
  auto base2ee = mLocalRobotState.getGlobalLinkTransform(mEEName).inverse();
  auto target2base = mLocalRobotState.getGlobalLinkTransform(frame);
  auto target2ee = base2ee * target2base;

  // Manually create adjoint matrix and apply transformation for twist
  // Note that the moveit set_diff_ik function will neglect all joint after the given tip frame
  // Thus the moveit function can be not used directly for base_frame (no joint to control)
  Eigen::MatrixXd adjoint = Eigen::ArrayXXd::Zero(6, 6);
  adjoint.block(0, 0, 3, 3) = target2ee.matrix().block(0, 0, 3, 3);
  adjoint.block(3, 3, 3, 3) = target2ee.matrix().block(0, 0, 3, 3);
  Eigen::Vector3d position = target2ee.matrix().block(0, 3, 3, 1);
  adjoint.block(0, 3, 3, 3) = skewSymmetric(position) * target2ee.matrix().block(0, 0, 3, 3);
  return adjoint * twist;
}
} // namespace sapien::ros2
