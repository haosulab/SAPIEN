#include "articulation/sapien_articulation.h"
#include "manager/sapien_controllable_articulation.h"
#include "utils/thread_safe_structure.hpp"

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <memory>
#include <spdlog/spdlog.h>
#include <utility>

namespace sapien::ros1 {

class SRobotHW : public hardware_interface::RobotHW {
private:
  ros::NodeHandlePtr mNode;
  SControllableArticulationWrapper *mWrapper;
  std::string mRobotName;
  uint32_t mDof;
  ros::Timer mTimer;

  ThreadSafeQueue<std::vector<physx::PxReal>> mWrapperPosition;
  ThreadSafeQueue<std::vector<physx::PxReal>> mWrapperVelocity;
  ThreadSafeQueue<ros::Time> mWrapperPositionTime;
  ThreadSafeQueue<ros::Time> mWrapperVelocityTime;

  hardware_interface::JointStateInterface mJointStateInterface;
  hardware_interface::PositionJointInterface mJointPositionInterface;
  hardware_interface::VelocityJointInterface mJointVelocityInterface;
  joint_limits_interface::PositionJointSaturationInterface mPositionJointLimitInterface;

  std::vector<double> mPositionCommand;
  std::vector<double> mVelocityCommand;
  std::vector<double> mPosition;
  std::vector<double> mVelocity;
  std::vector<double> mEffort;

  std::unique_ptr<controller_manager::ControllerManager> mControllerManager = nullptr;

public:
  SRobotHW(ros::NodeHandlePtr &node, SControllableArticulationWrapper *wrapper,
           std::string robotName, float frequency)
      : mNode(node), mWrapper(wrapper), mRobotName(std::move(robotName)) {
    auto jointNames = wrapper->getDriveJointNames();
    mDof = jointNames.size();
    wrapper->registerPositionCommands(&mWrapperPosition, jointNames, &mWrapperPositionTime);
    wrapper->registerVelocityCommands(&mWrapperVelocity, jointNames, &mWrapperVelocityTime);

    mPosition.resize(mDof);
    mVelocity.resize(mDof);
    mEffort.resize(mDof);
    mPositionCommand.resize(mDof);
    mVelocityCommand.resize(mDof, 0);

    for (uint32_t j = 0; j < mDof; ++j) {
      // Create joint state interface
      hardware_interface::JointStateHandle jointStateHandle(jointNames[j], &mPosition[j],
                                                            &mVelocity[j], &mEffort[j]);
      mJointStateInterface.registerHandle(jointStateHandle);

      // Create position joint interface
      hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &mPositionCommand[j]);
      joint_limits_interface::JointLimits limits;
      joint_limits_interface::getJointLimits(jointNames[j], *mNode, limits);
      joint_limits_interface::PositionJointSaturationHandle jointLimitsHandle(jointPositionHandle,
                                                                              limits);
      mPositionJointLimitInterface.registerHandle(jointLimitsHandle);
      mJointPositionInterface.registerHandle(jointPositionHandle);

      // Create velocity joint interface
      hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &mVelocityCommand[j]);
      mJointVelocityInterface.registerHandle(jointVelocityHandle);
    }
    // Register robot-level interface
    registerInterface(&mJointStateInterface);
    registerInterface(&mJointPositionInterface);
    registerInterface(&mJointVelocityInterface);
    mControllerManager = std::make_unique<controller_manager::ControllerManager>(this, *mNode);

    // Add timer loop
    ros::Duration interval(1 / frequency);
    mTimer = mNode->createTimer(interval, &SRobotHW::update, this, false, true);
    assert(mTimer.hasStarted());
  }

  void read(const ros::Time &time, const ros::Duration &period) {
    auto position = mWrapper->mJointPositions.read();
    auto velocity = mWrapper->mJointVelocities.read();
    mPosition.assign(position.begin(), position.end());
    //    mPositionCommand.assign(position.begin(), position.end());
    mVelocity.assign(velocity.begin(), velocity.end());
    auto qf = mWrapper->mArticulation->getQf();
    for (size_t j = 0; j < qf.size(); ++j) {
      mEffort[j] = velocity[j] * qf[j];
    }
  }

  void write(const ros::Time &time, const ros::Duration &period) {
    mPositionJointLimitInterface.enforceLimits(period);
    mWrapperPosition.push(
        std::vector<physx::PxReal>(mPositionCommand.begin(), mPositionCommand.end()));
    mWrapperVelocity.push(
        std::vector<physx::PxReal>(mVelocityCommand.begin(), mVelocityCommand.end()));
    mWrapperPositionTime.push(time);
    mWrapperVelocityTime.push(time);
  }

  void update(const ros::TimerEvent &event) {
    ros::Time currentReal = event.current_real;
    ros::Duration elapsed = currentReal - event.last_real;
    read(currentReal, elapsed);
    mControllerManager->update(currentReal, elapsed);
    write(currentReal, elapsed);
  }

  void start() {
    auto position = mWrapper->mJointPositions.read();
    mPositionCommand.assign(position.begin(), position.end());
    for (double &j : mVelocityCommand) {
      j = 0;
    }
  }
};
} // namespace sapien::ros1
