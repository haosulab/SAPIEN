//
// Created by sim on 10/2/19.
//

#include "controllable_articulation_wrapper.h"
#include <algorithm>
#include <iostream>

// Controllable articulation wrapper
ControllableArticulationWrapper::ControllableArticulationWrapper(
    IArticulationDrivable *articulation)
    : articulation(articulation) {
  jointNames = articulation->get_drive_joint_name();
  driveQpos.resize(jointNames.size(), 0);
}
bool ControllableArticulationWrapper::add_position_controller(
    const std::vector<std::string> &controllerJointNames, ThreadSafeQueue *queue) {
  // Get joint index mapping from controller to articulation
  std::vector<uint32_t> controllerIndex = {};
  for (const auto &qName : controllerJointNames) {
    auto iterator = std::find(jointNames.begin(), jointNames.end(), qName);
    if (iterator == jointNames.end()) {
      std::cerr << "Joint name given controller not found in the articulation: " << qName
                << std::endl;
      return false;
    }
    auto index = iterator - jointNames.begin();
    controllerIndex.push_back(index);
  }
  // Add position based controller based queue
  positionControllerIndexList.push_back(controllerIndex);
  positionControllerQueueList.push_back(queue);
  return true;
}
bool ControllableArticulationWrapper::add_velocity_controller(
    const std::vector<std::string> &controllerJointNames, ThreadSafeQueue *queue) {
  // Get joint index mapping from controller to articulation
  std::vector<uint32_t> controllerIndex = {};
  for (const auto &qName : controllerJointNames) {
    auto iterator = std::find(jointNames.begin(), jointNames.end(), qName);
    if (iterator == jointNames.end()) {
      std::cerr << "Joint name given controller not found in the articulation: " << qName
                << std::endl;
      return false;
    }
    auto index = iterator - jointNames.begin();
    controllerIndex.push_back(index);
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
ThreadSafeQueue *ControllableArticulationWrapper::get_joint_state_queue() {
  return jointStateQueue.get();
}
void ControllableArticulationWrapper::update(physx::PxReal timestep) {
  driveQpos = articulation->get_qpos();
  driveFromVelocityController(timestep);
  driveFromPositionController();
  if (controllerActive) {
    articulation->set_drive_target(driveQpos);
    controllerActive = false;
  }

  updateJointState();
}
std::vector<std::string> ControllableArticulationWrapper::get_drive_joint_name() {
  return articulation->get_drive_joint_name();
}
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
