//
// Created by sim on 10/2/19.
//
#pragma once
#include "articulation_interface.h"
#include <memory>
#include <mutex>
#include <queue>

namespace sapien {
class ThreadSafeQueue {
  std::mutex mLock;
  std::queue<std::vector<float>> mQueue;

public:
  ThreadSafeQueue();

  void push(const std::vector<float> &vec);
  void pushValue(const std::vector<float> vec);
  std::vector<float> pop();
  bool empty();
  void clear();
};

class ControllableArticulationWrapper {
private:
  std::unique_ptr<ThreadSafeQueue> jointStateQueue = std::make_unique<ThreadSafeQueue>();
  std::vector<ThreadSafeQueue *> positionControllerQueueList = {};
  std::vector<std::vector<uint32_t>> positionControllerIndexList = {};
  std::vector<ThreadSafeQueue *> velocityControllerQueueList = {};
  std::vector<std::vector<uint32_t>> velocityControllerIndexList = {};

  std::vector<physx::PxReal> driveQpos;
  bool controllerActive = false;

  // Cache
  std::vector<std::string> jointNames;
  float timestep=0.01;

public:
  IArticulationDrivable *articulation;
  bool needUpdateTimeStep = false;

private:
  void updateJointState();
  void driveFromPositionController();
  void driveFromVelocityController(physx::PxReal timestep);

public:
  explicit ControllableArticulationWrapper(IArticulationDrivable *articulation);
  bool add_position_controller(const std::vector<std::string> &controllerJointNames,
                               ThreadSafeQueue *queue);
  bool add_velocity_controller(const std::vector<std::string> &controllerJointNames,
                               ThreadSafeQueue *queue);

  ThreadSafeQueue *get_joint_state_queue();
  std::vector<std::string> get_drive_joint_name();
  void update(physx::PxReal timestep);
  void updateTimeStep(float timestep);
  float informMangerTimestepChange();
};

} // namespace sapien
