#pragma once

#include "articulation/sapien_articulation.h"
#include "articulation/sapien_articulation_base.h"
#include "articulation/sapien_joint.h"
#include "event_system/event_system.h"
#include <iostream>
#include <queue>
#include <spdlog/spdlog.h>
#include <vector>

#define CHECK_VEC_SIZE(v, n)                                                                      \
  {                                                                                               \
    if ((v).size() != n) {                                                                        \
      spdlog::error("Input vector size does not match DOF of articulation");                      \
      return;                                                                                     \
    }                                                                                             \
  }

namespace sapien::ros2 {

class RobotManager;

template <typename T> class ThreadSafeVector {
public:
  ThreadSafeVector(uint32_t num) : mVec(num){};
  void write(const std::vector<T> &input) {
    std::lock_guard<std::mutex> guard(mLock);
    if (input.size() != mVec.size()) {
      spdlog::warn("Attempt to write a thread safe vector with a vector of differernt size!");
      return;
    }
    for (int i = 0; i < mVec.size(); ++i) {
      mVec[i] = input[i];
    }
  };

  std::vector<T> read() {
    std::lock_guard<std::mutex> guard(mLock);
    std::vector<T> result(mVec);
    return result;
  };

private:
  std::vector<T> mVec;
  std::mutex mLock;
};

template <typename T> class ThreadSafeQueue {
public:
  ThreadSafeQueue() : mQueue(){};
  bool empty() {
    std::lock_guard<std::mutex> guard(mLock);
    return mQueue.empty();
  };
  void clear() {
    std::lock_guard<std::mutex> guard(mLock);
    std::queue<T> emptyQueue;
    std::swap(mQueue, emptyQueue);
  };
  T pop() {
    std::lock_guard<std::mutex> guard(mLock);
    T topElement = mQueue.front();
    mQueue.pop();
    return topElement;
  };
  void push(const T &element) {
    std::lock_guard<std::mutex> guard(mLock);
    mQueue.push(element);
  }

private:
  std::mutex mLock;
  std::queue<T> mQueue;
};

class SControllableArticulation : public IEventListener<EventStep> {
  friend RobotManager;

protected:
  SArticulation *mArticulation;

  // Cache
  std::vector<std::string> mQNames;
  std::vector<SJoint *> mJoints;
  float mTimeStep = 0;

public:
  // State
  ThreadSafeVector<float> mJointPositions;
  ThreadSafeVector<float> mJointVelocities;
  std::vector<ThreadSafeQueue<std::vector<float>> *> mPositionCommands;
  std::vector<ThreadSafeQueue<std::vector<float>> *> mVelocityCommands;
  std::vector<std::vector<uint32_t>> mPositionCommandsIndex;
  std::vector<std::vector<uint32_t>> mVelocityCommandsIndex;

public:
  explicit SControllableArticulation(SArticulation *articulation)
      : mArticulation(articulation), mJointPositions(articulation->dof()),
        mJointVelocities(articulation->dof()) {
    auto joints = mArticulation->getSJoints();
    for (int i = 0; i < joints.size(); ++i) {
      if (joints[i]->getDof() > 0) {
        mJoints.push_back(joints[i]);
      }
      for (int j = 0; j < joints[i]->getDof(); ++j) {
        mQNames.push_back(joints[i]->getName());
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
        std::cerr << "Joint name given controller not found in the articulation: " << qName
                  << std::endl;
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
        std::cerr << "Joint name given controller not found in the articulation: " << qName
                  << std::endl;
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
    std::vector<float> currentVelocityCommands(mArticulation->dof(), -2000);
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
      if (currentVelocityCommands[l] > -999) {
        if (currentPositionCommands[l] > -999) {
          currentPositionCommands[l] += currentVelocityCommands[l] * mTimeStep;
        } else {
          currentPositionCommands[l] = lastDriveTarget[l] + currentVelocityCommands[l] * mTimeStep;
        }
      }
      uint32_t i = 0;
      for (auto &j : mJoints) {
        for (auto axis : j->getAxes()) {
          // We use -1000 to represent inactive control command
          if (currentPositionCommands[i] > -999) {
            j->getPxJoint()->setDriveTarget(axis, currentPositionCommands[i]);
          }
          i += 1;
        }
      }
    }

    if (!mVelocityCommands.empty()) {
      uint32_t i = 0;
      for (auto &j : mJoints) {
        for (auto axis : j->getAxes()) {
          // We use -1000 to represent inactive control command
          if (currentVelocityCommands[i] > -999) {
            j->getPxJoint()->setDriveVelocity(axis, currentVelocityCommands[i]);
          }
          i += 1;
        }
      }
    }
  };

public:
};
} // namespace sapien::ros2
