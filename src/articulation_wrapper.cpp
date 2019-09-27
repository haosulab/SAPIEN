#include "articulation_wrapper.h"
#include <iostream>
#include <sstream>
#include <cassert>

void PxArticulationWrapper::updateCache() {
  articulation->copyInternalStateToCache(*cache, PxArticulationCache::eALL);
}

void PxArticulationWrapper::updateArticulation() {
  articulation->applyCache(*cache, PxArticulationCache::eALL);
}

EArticulationType PxArticulationWrapper::get_articulation_type() const {
  return DYNAMIC_ARTICULATION;
}

uint32_t PxArticulationWrapper::dof() const { return articulation->getDofs(); }

std::vector<std::string> PxArticulationWrapper::get_joint_names() const { return jointNames; }

std::vector<uint32_t> PxArticulationWrapper::get_joint_dofs() const { return jointDofs; }

std::vector<std::tuple<physx::PxReal, physx::PxReal>>
PxArticulationWrapper::get_joint_limits() const {
  return jointLimits;
}

std::vector<physx::PxReal> PxArticulationWrapper::get_qpos() const {
  return std::vector<physx::PxReal>(cache->jointPosition, cache->jointPosition + dof());
}

void PxArticulationWrapper::set_qpos(const std::vector<physx::PxReal> &v) {
  assert(v.size() == dof());
  for (size_t i = 0; i < v.size(); ++i) {
    cache->jointPosition[i] = v[i];
  }
}

std::vector<physx::PxReal> PxArticulationWrapper::get_qvel() const {
  return std::vector<physx::PxReal>(cache->jointVelocity, cache->jointVelocity + dof());
}

void PxArticulationWrapper::set_qvel(const std::vector<physx::PxReal> &v) {
  assert(v.size() == dof());
  for (size_t i = 0; i < v.size(); ++i) {
    cache->jointVelocity[i] = v[i];
  }
}

std::vector<physx::PxReal> PxArticulationWrapper::get_qacc() const {
  return std::vector<physx::PxReal>(cache->jointAcceleration, cache->jointAcceleration + dof());
}
void PxArticulationWrapper::set_qacc(const std::vector<physx::PxReal> &v) {
  assert(v.size() == dof());
  for (size_t i = 0; i < v.size(); ++i) {
    cache->jointAcceleration[i] = v[i];
  }
}

std::vector<physx::PxReal> PxArticulationWrapper::get_qf() const {
  return std::vector<physx::PxReal>(cache->jointForce, cache->jointForce + dof());
}
void PxArticulationWrapper::set_qf(const std::vector<physx::PxReal> &v) {
  assert(v.size() == dof());
  for (size_t i = 0; i < v.size(); ++i) {
    cache->jointForce[i] = v[i];
  }
}
