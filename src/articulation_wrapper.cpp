#include "articulation_wrapper.h"
#include <iostream>
#include <sstream>
#include <cassert>

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

}
