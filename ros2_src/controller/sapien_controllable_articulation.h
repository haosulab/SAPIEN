#pragma once

#include <PxPhysicsAPI.h>
#include "event_system/event_system.h"
#include "utils/thread_safe_structure.hpp"
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <vector>

namespace sapien {
class SArticulation;
class SJoint;
} // namespace sapien
namespace sapien::ros2 {

class RobotManager;

class SControllableArticulationWrapper : public IEventListener<EventStep> {
  friend RobotManager;

protected:
  SArticulation *mArticulation;

  // Cache
  std::vector<std::string> mQNames;
  std::vector<SJoint *> mJoints;
  float mTimeStep = 0;
  rclcpp::Clock::SharedPtr mClock = nullptr;

public:
  // State
  ThreadSafeVector<float> mJointPositions;
  ThreadSafeVector<float> mJointVelocities;

  std::vector<ThreadSafeQueue<std::vector<float>> *> mPositionCommands;
  std::vector<ThreadSafeQueue<std::vector<float>> *> mVelocityCommands;
  std::vector<ThreadSafeVector<float> *> mContinuousVelocityCommands;

  std::vector<std::vector<uint32_t>> mPositionCommandsIndex;
  std::vector<std::vector<uint32_t>> mVelocityCommandsIndex;
  std::vector<std::vector<uint32_t>> mContinuousVelocityCommandsIndex;

  std::vector<ThreadSafeQueue<rclcpp::Time> *> mPositionCommandsTimer;
  std::vector<ThreadSafeQueue<rclcpp::Time> *> mVelocityCommandsTimer;
  std::vector<ThreadSafeQueue<rclcpp::Time> *> mContinuousVelocityCommandsTimer;

public:
  explicit SControllableArticulationWrapper(SArticulation *articulation,
                                            rclcpp::Clock::SharedPtr clock);

  inline std::vector<std::string> getDriveJointNames() { return mQNames; }

  bool registerPositionCommands(ThreadSafeQueue<std::vector<float>> *command,
                                const std::vector<std::string> &commandedJointNames,
                                ThreadSafeQueue<rclcpp::Time> *commandTimer);

  bool registerVelocityCommands(ThreadSafeQueue<std::vector<float>> *command,
                                const std::vector<std::string> &commandedJointNames,
                                ThreadSafeQueue<rclcpp::Time> *commandTimer);

  bool registerContinuousVelocityCommands(ThreadSafeVector<float> *command,
                                          const std::vector<std::string> &commandedJointNames,
                                          ThreadSafeQueue<rclcpp::Time> *commandTimer);

protected:
  void onEvent(EventStep &event) override;

public:
};
} // namespace sapien::ros2
