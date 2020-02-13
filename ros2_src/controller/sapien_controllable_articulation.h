#pragma once

#include "articulation/sapien_articulation.h"
#include "articulation/sapien_articulation_base.h"
#include "articulation/sapien_joint.h"
#include "event_system/event_system.h"
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

public:
  // State
  ThreadSafeVector<float> mJointPositions;
  ThreadSafeVector<float> mJointVelocities;
  ThreadSafeQueue<std::vector<float>> mPositionCommands;
  ThreadSafeQueue<std::vector<float>> mVelocityCommands;

public:
  explicit SControllableArticulation(SArticulation *articulation)
      : mArticulation(articulation), mJointPositions(articulation->dof()),
        mJointVelocities(articulation->dof()){
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

protected:
  void onEvent(EventStep &event) override {
    mJointPositions.write(mArticulation->getQpos());
    mJointVelocities.write(mArticulation->getQvel());
    if (!mPositionCommands.empty()) {
      auto command = mPositionCommands.pop();
      CHECK_VEC_SIZE(command, mArticulation->dof());
      uint32_t i = 0;
      for (auto &j : mJoints) {
        for (auto axis : j->getAxes()) {
          // We use -1000 to represent inactive control command
          if (command[i] > -999) {
            j->getPxJoint()->setDriveTarget(axis, command[i]);
          }
          i += 1;
        }
      }
    }
    if (!mVelocityCommands.empty()) {
      auto command = mVelocityCommands.pop();
      CHECK_VEC_SIZE(command, mArticulation->dof());
      uint32_t i = 0;
      for (auto &j : mJoints) {
        for (auto axis : j->getAxes()) {
          // We use -1000 to represent inactive control command
          if (command[i] > -999) {
            j->getPxJoint()->setDriveVelocity(axis, command[i]);
          }
          i += 1;
        }
      }
    }
  };

public:
};
} // namespace sapien::ros2
