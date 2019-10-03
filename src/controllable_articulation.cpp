//
// Created by sim on 10/2/19.
//

#include "controllable_articulation.h"
#include <iostream>

// Thread Safe Queue
ThreadSafeQueue::ThreadSafeQueue() { mQueue = std::queue<std::vector<float>>(); }
void ThreadSafeQueue::push(const std::vector<float> &vec) {
  std::lock_guard<std::mutex> guard(mLock);
  mQueue.push(vec);
}
std::vector<float> ThreadSafeQueue::pop() {
  std::lock_guard<std::mutex> guard(mLock);
  std::vector<float> vec = mQueue.front();
  mQueue.pop();
  return vec;
}
bool ThreadSafeQueue::empty() {
  std::lock_guard<std::mutex> guard(mLock);
  return mQueue.empty();
}
void ThreadSafeQueue::clear() {
  std::lock_guard<std::mutex> guard(mLock);
  std::queue<std::vector<float>> empty;
  std::swap(mQueue, empty);
}
void ThreadSafeQueue::pushValue(const std::vector<float> vec) {
  std::lock_guard<std::mutex> guard(mLock);
  mQueue.push(vec);
}

// Controllable articulation wrapper
ControllableArticulationWrapper::ControllableArticulationWrapper(
    IArticulationDrivable *articulation)
    : articulation(articulation) {
  jointNames = articulation->get_joint_names();
}
bool ControllableArticulationWrapper::add_position_controller(
    const std::vector<std::string> &jointName, ThreadSafeQueue *queue) {
  // Get joint index mapping from controller to articulation
  std::vector<uint32_t> controllerIndex = {};
  for (const auto &qName : jointName) {
    bool found = false;
    for (size_t j = 0; j < jointNames.size(); ++j) {
      if (qName == jointNames[j]) {
        controllerIndex.push_back(j);
        found = true;
        break;
      }
    }
    if (!found) {
      std::cerr << "Joint name not found: " << qName << std::endl;
      return false;
    }
  }
  // Add position based controller based queue
  positionControllerIndexList.push_back(controllerIndex);
  positionControllerQueueList.push_back(queue);
  return true;
}
bool ControllableArticulationWrapper::add_velocity_controller(
    const std::vector<std::string> &jointName, ThreadSafeQueue *queue) {
  // Get joint index mapping from controller to articulation
  std::vector<uint32_t> controllerIndex = {};
  auto articulationJointNames = articulation->get_joint_names();
  for (const auto &qName : jointName) {
    bool found = false;
    for (size_t j = 0; j < articulationJointNames.size(); ++j) {
      if (qName == articulationJointNames[j]) {
        controllerIndex.push_back(j);
        found = true;
        break;
      }
    }
    if (!found) {
      std::cerr << "Joint name not found: " << qName << std::endl;
      return false;
    }
  }
  // Add position based controller based queue
  velocityControllerIndexList.push_back(controllerIndex);
  velocityControllerQueueList.push_back(queue);
  return true;
}
void ControllableArticulationWrapper::updateJointState() {
  // TODO: update acceleration
  std::vector<physx::PxReal> jointState = articulation->get_qpos();
  std::vector<physx::PxReal> qvel = articulation->get_qvel();
  jointState.insert(jointState.end(), qvel.begin(), qvel.end());
  jointStateQueue->push(jointState);
}
void ControllableArticulationWrapper::driveFromPositionController() {
  for (size_t i = 0; i < positionControllerQueueList.size(); ++i) {
    // If no controller give the signal, continue for next one
    if (positionControllerQueueList[i]->empty()) {
      continue;
    }
    auto controllerIndex = positionControllerIndexList[i];
    auto queue = positionControllerQueueList[i]->pop();
    for (size_t j = 0; j < controllerIndex.size(); ++j) {
      driveQpos[controllerIndex[j]] = queue[j];
    }
    controllerActive = true;
  }
}
void ControllableArticulationWrapper::driveFromVelocityController(physx::PxReal timestep) {
  for (size_t i = 0; i < velocityControllerQueueList.size(); ++i) {
    // If no controller give the signal, continue for next one
    if (velocityControllerQueueList[i]->empty()) {
      continue;
    }
    auto controllerIndex = velocityControllerIndexList[i];
    auto queue = velocityControllerQueueList[i]->pop();
    for (size_t j = 0; j < controllerIndex.size(); ++j) {
      driveQpos[controllerIndex[j]] += queue[j] * timestep;
    }
    controllerActive = true;
  }
}
ThreadSafeQueue *ControllableArticulationWrapper::getJointStateQueue() {
  return jointStateQueue.get();
}
void ControllableArticulationWrapper::update(physx::PxReal timestep) {
  driveFromVelocityController(timestep);
  driveFromPositionController();
  if (controllerActive) {
    articulation->set_drive_target(driveQpos);
  }

  updateJointState();
}
