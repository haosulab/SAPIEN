#include "articulation_wrapper.h"
#include <algorithm>
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

std::vector<std::array<physx::PxReal, 2>> ArticulationWrapper::get_joint_limits() const {
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
    activeJoints[i]->setDriveTarget(jointAxises[i], v[i]);
  }
}
void ArticulationWrapper::set_drive_property(PxReal stiffness, PxReal damping, PxReal forceLimit,
                                             const std::vector<uint32_t> &jointIndex) {
  // If no index is given, then set for all joint
  std::vector<uint32_t> index;
  if (jointIndex.empty()) {
    index.resize(activeJoints.size());
    std::iota(std::begin(index), std::end(index), 0);
  } else {
    index = jointIndex;
  }

  for (unsigned int i : index) {
    auto joint = activeJoints[i];
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
    std::vector<PxReal> otherCache(cache->jointForce, cache->jointForce + dof());
    articulation->computeGeneralizedExternalForce(*cache);
    for (size_t i = 0; i < dof(); ++i) {
      cache->jointForce[i] += gravityCache[i];
      cache->jointForce[i] += otherCache[i];
    }
    articulation->applyCache(*cache, PxArticulationCache::eFORCE);
  }
}

// Force actuator
void ArticulationWrapper::addForceActuator(const std::string &name, PxReal lowerLimit,
                                           PxReal upperLimit) {
  if (balanceForce) {
    std::cerr << "Could not add actuator to a balanced force articulation! This tag should only "
                 "be used for robot"
              << std::endl;
    return;
  }
  auto index = std::find(jointNamesDOF.begin(), jointNamesDOF.end(), name) - jointNamesDOF.begin();
  if (index == jointNamesDOF.size()) {
    std::cerr << "Could not find joint name in movable joints: " << name.c_str()
              << "\nMay be you add actuator on a fixed joint?" << std::endl;
    return;
  }
  forceActuatorName.push_back(name);
  forceActuatorLimit.push_back({lowerLimit, upperLimit});
  forceActuatorIndex.push_back(index);
}
std::vector<std::array<PxReal, 2>> const &ArticulationWrapper::getForceActuatorRanges() const {
  return forceActuatorLimit;
}
std::vector<std::string> const &ArticulationWrapper::getForceActuatorNames() const {
  return forceActuatorName;
}
void ArticulationWrapper::applyActuatorForce(const std::vector<physx::PxReal> &v) {
  assert(v.size() == forceActuatorName.size());
  for (size_t i = 0; i < v.size(); ++i) {
    auto [lowerLimit, upperLimit] = forceActuatorLimit[i];
    auto force = v[i];
    force = force > upperLimit ? upperLimit : force;
    force = force < lowerLimit ? lowerLimit : force;
    cache->jointForce[forceActuatorIndex[i]] = force;
  }
  articulation->applyCache(*cache, PxArticulationCache::eFORCE);
}
std::vector<std::array<PxReal, 6>> ArticulationWrapper::get_cfrc_ext() {
  std::vector<std::array<PxReal, 6>> xf(links.size());
  for (size_t i = 0; i < links.size(); ++i) {
    auto v = cache->linkAcceleration[i].linear;
    auto w = cache->linkAcceleration[i].angular;
    auto mass = linkMasses[i];
    auto inertial = linkInertial[i];
    xf[i] = {v[0] * mass,        v[1] * mass,        v[2] * mass,
             inertial[0] * w[0], inertial[1] * w[1], inertial[2] * w[2]};
  }
  return xf;
}

std::vector<physx::PxRigidBody *> ArticulationWrapper::get_links() const {
  return std::vector<physx::PxRigidBody *>(links.begin(), links.end());
}

std::vector<std::string> ArticulationWrapper::get_link_names() const {
  std::vector<std::string> names;
  for (auto link : links) {
    names.emplace_back(link->getName());
  }
  return names;
}

std::vector<physx_id_t> ArticulationWrapper::get_link_ids() const { return linkSegmentationIds; }
void ArticulationWrapper::move_base(const PxTransform &newT) {
  articulation->teleportRootLink(newT, true);
}

} // namespace sapien
