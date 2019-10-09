#include "articulation_wrapper.h"
#include <cassert>
#include <iostream>
#include <numeric>
#include <sstream>

namespace sapien {
void ArticulationWrapper::updateCache() {
  articulation->copyInternalStateToCache(*cache, PxArticulationCache::eALL);
}

void ArticulationWrapper::updateArticulation() {
  articulation->applyCache(*cache, PxArticulationCache::eALL);
}

EArticulationType ArticulationWrapper::get_articulation_type() const {
  return DYNAMIC_ARTICULATION;
}

uint32_t ArticulationWrapper::dof() const { return articulation->getDofs(); }

std::vector<std::string> ArticulationWrapper::get_joint_names() const { return jointNames; }

std::vector<uint32_t> ArticulationWrapper::get_joint_dofs() const { return jointDofs; }

std::vector<std::tuple<physx::PxReal, physx::PxReal>>
ArticulationWrapper::get_joint_limits() const {
  return jointLimits;
}

std::vector<physx::PxReal> ArticulationWrapper::get_qpos() const {
  return std::vector<physx::PxReal>(cache->jointPosition, cache->jointPosition + dof());
}

void ArticulationWrapper::set_qpos(const std::vector<physx::PxReal> &v) {
  assert(v.size() == dof());
  for (size_t i = 0; i < v.size(); ++i) {
    cache->jointPosition[i] = v[i];
  }
  articulation->applyCache(*cache, PxArticulationCache::ePOSITION);
}

std::vector<physx::PxReal> ArticulationWrapper::get_qvel() const {
  return std::vector<physx::PxReal>(cache->jointVelocity, cache->jointVelocity + dof());
}

void ArticulationWrapper::set_qvel(const std::vector<physx::PxReal> &v) {
  assert(v.size() == dof());
  for (size_t i = 0; i < v.size(); ++i) {
    cache->jointVelocity[i] = v[i];
  }
  articulation->applyCache(*cache, PxArticulationCache::eVELOCITY);
}

std::vector<physx::PxReal> ArticulationWrapper::get_qacc() const {
  return std::vector<physx::PxReal>(cache->jointAcceleration, cache->jointAcceleration + dof());
}
void ArticulationWrapper::set_qacc(const std::vector<physx::PxReal> &v) {
  assert(v.size() == dof());
  for (size_t i = 0; i < v.size(); ++i) {
    cache->jointAcceleration[i] = v[i];
  }
  articulation->applyCache(*cache, PxArticulationCache::eACCELERATION);
}

std::vector<physx::PxReal> ArticulationWrapper::get_qf() const {
  return std::vector<physx::PxReal>(cache->jointForce, cache->jointForce + dof());
}
void ArticulationWrapper::set_qf(const std::vector<physx::PxReal> &v) {
  assert(v.size() == dof());
  for (size_t i = 0; i < v.size(); ++i) {
    cache->jointForce[i] = v[i];
  }
  articulation->applyCache(*cache, PxArticulationCache::eFORCE);
}
std::vector<std::string> ArticulationWrapper::get_drive_joint_names() const {
  return jointNamesDOF;
}
void ArticulationWrapper::set_drive_target(const std::vector<physx::PxReal> &v) {
  assert(v.size() == dof());
  for (size_t i = 0; i < v.size(); ++i) {
    joints[i]->setDriveTarget(jointAxises[i], v[i]);
  }
}
void ArticulationWrapper::set_drive_property(PxReal stiffness, PxReal damping, PxReal forceLimit,
                                             const std::vector<uint32_t> &jointIndex) {
  // If no index is given, then set for all joint
  std::vector<uint32_t> index;
  if (jointIndex.empty()) {
    index.resize(joints.size());
    std::iota(std::begin(index), std::end(index), 0);
  } else {
    index = jointIndex;
  }

  for (unsigned int i : index) {
    auto joint = joints[i];
    joint->setDrive(jointAxises[i], stiffness, damping, forceLimit);
  }
}
void ArticulationWrapper::set_force_balance(bool balance) { balanceForce = balance; }
void ArticulationWrapper::update() {
  updateCache();

  // Balance passive force
  if (balanceForce) {
    articulation->commonInit();
    articulation->computeGeneralizedGravityForce(*cache);
    std::vector<PxReal> gravityCache(cache->jointForce, cache->jointForce + dof());
    articulation->copyInternalStateToCache(*cache, PxArticulationCache::eVELOCITY);
    articulation->computeCoriolisAndCentrifugalForce(*cache);
    for (size_t i = 0; i < dof(); ++i) {
      cache->jointForce[i] += gravityCache[i];
    }
    articulation->applyCache(*cache, PxArticulationCache::eFORCE);
  }
}

} // namespace sapien
