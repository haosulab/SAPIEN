#pragma once

#include "articulation/sapien_articulation.h"
#include "articulation/sapien_joint.h"
#include "event_system/event_system.h"
#include "utils/thread_safe_structure.hpp"
#include <rclcpp/rclcpp.hpp>
#include <utility>

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

public:
  explicit SControllableArticulationWrapper(SArticulation *articulation,
                                            rclcpp::Clock::SharedPtr clock)
      : mArticulation(articulation), mClock(std::move(clock)),
        mJointPositions(articulation->dof()), mJointVelocities(articulation->dof()) {
    auto joints = mArticulation->getSJoints();
    for (auto &joint : joints) {
      if (joint->getDof() > 0) {
        mJoints.push_back(joint);
      }
      for (uint32_t j = 0; j < joint->getDof(); ++j) {
        mQNames.push_back(joint->getName());
      }
    }
  };

  inline std::vector<std::string> getDriveJointNames() { return mQNames; }

  bool registerPositionCommands(ThreadSafeQueue<std::vector<float>> *command,
                                const std::vector<std::string> &commandedJointNames) {
    std::vector<uint32_t> controllerIndex = {};
    for (const auto &qName : commandedJointNames) {
      auto iterator = std::find(mQNames.begin(), mQNames.end(), qName);
      if (iterator == mQNames.end()) {
        spdlog::warn("Joint name given not found in articulation %s \n", qName.c_str());
        return false;
      }
      auto index = iterator - mQNames.begin();
      controllerIndex.push_back(index);
    }

    // Register position based command
    mPositionCommandsIndex.push_back(controllerIndex);
    mPositionCommands.push_back(command);
    return true;
  };

  bool registerVelocityCommands(ThreadSafeQueue<std::vector<float>> *command,
                                const std::vector<std::string> &commandedJointNames) {
    std::vector<uint32_t> controllerIndex = {};
    for (const auto &qName : commandedJointNames) {
      auto iterator = std::find(mQNames.begin(), mQNames.end(), qName);
      if (iterator == mQNames.end()) {
        spdlog::warn("Joint name given not found in articulation %s \n", qName.c_str());
        return false;
      }
      auto index = iterator - mQNames.begin();
      controllerIndex.push_back(index);
    }

    // Register velocity based command
    mVelocityCommandsIndex.push_back(controllerIndex);
    mVelocityCommands.push_back(command);
    return true;
  };
  bool registerContinuousVelocityCommands(ThreadSafeVector<float> *command,
                                          const std::vector<std::string> &commandedJointNames) {
    std::vector<uint32_t> controllerIndex = {};
    for (const auto &qName : commandedJointNames) {
      auto iterator = std::find(mQNames.begin(), mQNames.end(), qName);
      if (iterator == mQNames.end()) {
        spdlog::warn("Joint name given not found in articulation %s \n", qName.c_str());
        return false;
      }
      auto index = iterator - mQNames.begin();
      controllerIndex.push_back(index);
    }

    // Register velocity based command
    mContinuousVelocityCommandsIndex.push_back(controllerIndex);
    mContinuousVelocityCommands.push_back(command);
    return true;
  };

protected:
  void onEvent(EventStep &event) override {
    mJointPositions.write(mArticulation->getQpos());
    mJointVelocities.write(mArticulation->getQvel());
    mTimeStep = event.timeStep;

    // Aggregate position commands information
    std::vector<float> currentPositionCommands(mArticulation->dof(), -2000);
    for (size_t k = 0; k < mPositionCommands.size(); ++k) {
      if (mPositionCommands[k]->empty()) {
        continue;
      }
      auto index = mPositionCommandsIndex[k];
      auto command = mPositionCommands[k]->pop();
      for (size_t i = 0; i < index.size(); ++i) {
        currentPositionCommands[index[i]] = command[i];
      }
    }

    // Aggregate velocity commands information
    std::vector<float> currentVelocityCommands(mArticulation->dof(), 0);
    for (size_t k = 0; k < mVelocityCommands.size(); ++k) {
      if (mVelocityCommands[k]->empty()) {
        continue;
      }
      auto index = mVelocityCommandsIndex[k];
      auto command = mVelocityCommands[k]->pop();
      for (size_t i = 0; i < index.size(); ++i) {
        currentVelocityCommands[index[i]] = command[i];
      }
    }

    // Integrate Position and Velocity Commands
    auto lastDriveTarget = mArticulation->getDriveTarget();
    for (size_t l = 0; l < currentPositionCommands.size(); ++l) {
      if (currentPositionCommands[l] > -999) {
        currentPositionCommands[l] += currentVelocityCommands[l] * mTimeStep;
      } else {
        currentPositionCommands[l] = lastDriveTarget[l] + currentVelocityCommands[l] * mTimeStep;
      }
    }

    // Execute position command
    if (!mPositionCommands.empty() || !mVelocityCommands.empty()) {
      uint32_t i = 0;
      for (auto &j : mJoints) {
        for (auto axis : j->getAxes()) {
          if (currentPositionCommands[i] > -999) {
            j->getPxJoint()->setDriveTarget(axis, currentPositionCommands[i]);
          }
          i += 1;
        }
      }
    }

    // Aggregate continuous velocity information to velocity command
    if (!mContinuousVelocityCommands.empty()) {
      for (size_t i = 0; i < mContinuousVelocityCommands.size(); ++i) {
        auto continuousVelocityCommands = mContinuousVelocityCommands[i]->read();
        auto index = mContinuousVelocityCommandsIndex[i];
        for (size_t j = 0; j < index.size(); ++j) {
          currentVelocityCommands[index[j]] += continuousVelocityCommands[j];
        }
      }
    }

    // Execute velocity command
    if (!mVelocityCommands.empty() || !mContinuousVelocityCommands.empty()) {
      uint32_t i = 0;
      for (auto &j : mJoints) {
        for (auto axis : j->getAxes()) {
          j->getPxJoint()->setDriveVelocity(axis, currentVelocityCommands[i]);
          i += 1;
        }
      }
    }
  };

public:
};
} // namespace sapien::ros2
