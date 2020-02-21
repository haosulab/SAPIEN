#pragma once

#include <utility>

#include "moveit/robot_state/robot_state.h"
#include "rclcpp/rclcpp.hpp"

#include "sapien_controllable_articulation.h"
#include "utils/thread_safe_structure.hpp"

#define COMPUTE_VELOCITY_AND_INTEGRATE(frame_name)                                                \
  {                                                                                               \
    mRobotState->computeVariableVelocity(mJointModelGroup, qdot, twist,                           \
                                         mJointModelGroup->getLinkModel(frame_name));             \
    foundIK = mRobotState->integrateVariableVelocity(mJointModelGroup, qdot, mTimeStep);          \
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
  CartesianVelocityController(const std::string &nameSpace, rclcpp::Node::SharedPtr node,
                              rclcpp::Clock::SharedPtr clock,
                              SControllableArticulationWrapper *wrapper,
                              robot_state::RobotState *robotState, const std::string &groupName,
                              const std::string &serviceName)
      : mNode(std::move(node)), mClock(std::move(clock)), mRobotState(robotState),
        mLocalRobotState(*robotState), mVelocityCommand() {
    mJointModelGroup = mLocalRobotState.getJointModelGroup(groupName);
    mJointNames = mJointModelGroup->getActiveJointModelNames();
    mEEName = mJointModelGroup->getLinkModelNames().back();
    mBaseName = mJointModelGroup->getLinkModelNames()[0];
    RCLCPP_INFO(mNode->get_logger(), "Initialize cartesian velocity controller. EE: %s, Base: %s",
                mEEName.c_str(), mBaseName.c_str());
    // Time step will be initialized when build it using robot manager
    // Do not worry if not initialized here

    // Register
    wrapper->registerVelocityCommands(&mVelocityCommand, mJointNames);
  };

  void moveCartesian(const std::array<float, 3> &vec, MoveType type) {
    if (!mLastStepCalled) {
      synchronizeRealRobotState();
    }
    bool foundIK = false;
    Eigen::VectorXd twist(6);
    Eigen::VectorXd qdot;
    switch (type) {
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
      RCLCPP_WARN(mNode->get_logger(), "IK not found.");
      return;
    }
    //    mLocalRobotState.copyJointGroupPositions(mJointModelGroup, mJointValues);
    //    mVelocityCommand.push(std::vector<float>(mJointValues.begin(), mJointValues.end()));
    std::vector<float> stepVelocity(qdot.data(),
                                    qdot.data() + mJointModelGroup->getVariableCount());
    mVelocityCommand.push(stepVelocity);

    // Update control bool
    mCurrentStepCalled = true;
  };

protected:
  void synchronizeRealRobotState() {
    // Copy global real time robot state to local robot state
    mRobotState->copyJointGroupPositions(mJointModelGroup, mJointValues);
    mLocalRobotState.setJointGroupPositions(mJointModelGroup, mJointValues);
  }

  Eigen::VectorXd transformTwist(const Eigen::VectorXd &twist, const std::string &frame) {
    // Rotate the jacobian to the end-effector frame
    auto base2ee = mLocalRobotState.getGlobalLinkTransform(mEEName).inverse();
    auto target2base = mLocalRobotState.getGlobalLinkTransform(frame);
    auto target2ee = base2ee * target2base;
    Eigen::MatrixXd adjoint = Eigen::ArrayXXd::Zero(6, 6);
    adjoint.block(0, 0, 3, 3) = target2ee.matrix().block(0, 0, 3, 3);
    adjoint.block(3, 3, 3, 3) = target2ee.matrix().block(0, 0, 3, 3);
    Eigen::Vector3d position = target2ee.matrix().block(0, 3, 3, 1);
    adjoint.block(0, 3, 3, 3) = skewSymmetric(position) * target2ee.matrix().block(0, 0, 3, 3);
    return adjoint * twist;
  };

  static inline Eigen::Matrix3d skewSymmetric(Eigen::Vector3d t) {
    Eigen::Matrix3d hatMatrix;
    hatMatrix << 0, -t(2), t(1), t(2), 0, -t(0), -t(1), t(0), 0;
    return hatMatrix;
  };
};

} // namespace sapien::ros2
