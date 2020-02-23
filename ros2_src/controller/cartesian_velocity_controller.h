#pragma once

#include <utility>

#include "moveit/robot_state/robot_state.h"
#include "rclcpp/rclcpp.hpp"

#include "sapien_ros2_communication_interface/srv/cartesian_velocity.hpp"
#include "utils/thread_safe_structure.hpp"

#define COMPUTE_VELOCITY_AND_INTEGRATE(frame_name)                                                \
  {                                                                                               \
    mRobotState->computeVariableVelocity(mJointModelGroup, jointVelocity, twist,                  \
                                         mJointModelGroup->getLinkModel(frame_name));             \
    foundIK = mRobotState->integrateVariableVelocity(mJointModelGroup, jointVelocity, mTimeStep); \
  }

namespace sapien::ros2 {
class SControllableArticulationWrapper;
class RobotManager;

enum CartesianCommand {
  X_F = 0,
  Y_F = 1,
  Z_F = 2,
  ROLL_F = 3,
  PITCH_F = 4,
  YAW_F = 5,
  X_B = 6,
  Y_B = 7,
  Z_B = 8,
  ROLL_B = 9,
  PITCH_B = 10,
  YAW_B = 11
};

enum MoveType { WorldTranslate, WorldRotate, LocalTranslate, LocalRotate };

class CartesianVelocityController {
  friend RobotManager;

public:
protected:
  rclcpp::Node::SharedPtr mNode;
  rclcpp::Clock::SharedPtr mClock;
  robot_state::RobotState *mRobotState = nullptr;
  robot_state::RobotState mLocalRobotState;
  const robot_state::JointModelGroup *mJointModelGroup;
  rclcpp::Service<sapien_ros2_communication_interface::srv::CartesianVelocity>::SharedPtr mService;

  // Cache
  std::vector<std::string> mJointNames;
  std::string mEEName;
  std::string mBaseName;
  std::vector<double> mJointValues;

  // Behavior Control
  double mTimeStep = 0;
  bool mCurrentStepCalled = false;
  bool mLastStepCalled = false;

  // Command
  ThreadSafeQueue<std::vector<float>> mVelocityCommand;

public:
  CartesianVelocityController(rclcpp::Node::SharedPtr node, rclcpp::Clock::SharedPtr clock,
                              SControllableArticulationWrapper *wrapper,
                              robot_state::RobotState *robotState, const std::string &groupName,
                              const std::string &serviceName);

  void moveCartesian(const std::array<float, 3> &vec, MoveType type);

protected:
  void synchronizeRealRobotState();

  void handleService(
      const std::shared_ptr<sapien_ros2_communication_interface::srv::CartesianVelocity_Request>
          req,
      std::shared_ptr<sapien_ros2_communication_interface::srv::CartesianVelocity_Response> res);

  Eigen::VectorXd transformTwist(const Eigen::VectorXd &twist, const std::string &frame);

  static inline Eigen::Matrix3d skewSymmetric(Eigen::Vector3d t) {
    Eigen::Matrix3d hatMatrix;
    hatMatrix << 0, -t(2), t(1), t(2), 0, -t(0), -t(1), t(0), 0;
    return hatMatrix;
  };
};

} // namespace sapien::ros2
