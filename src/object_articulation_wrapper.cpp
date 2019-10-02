#include "object_articulation_wrapper.h"
#include <iostream>
#include <sstream>
#include <cassert>


EArticulationType PxObjectWrapper::get_articulation_type() const {
  return OBJECT_ARTICULATION;
}

uint32_t PxObjectWrapper::dof() const { return obj->getDofs(); }

std::vector<std::string> PxObjectWrapper::get_joint_names() const { return obj->getJointNames(); }

std::vector<uint32_t> PxObjectWrapper::get_joint_dofs() const { return obj->getJointDofs(); }

std::vector<std::tuple<physx::PxReal, physx::PxReal>>
PxObjectWrapper::get_joint_limits() const {
  return obj->getJointLimits();
}

std::vector<physx::PxReal> PxObjectWrapper::get_qpos() const {
  return obj->getJointPositions();
}

void PxObjectWrapper::set_qpos(const std::vector<physx::PxReal> &v) {}

std::vector<physx::PxReal> PxObjectWrapper::get_qvel() const {
  return obj->getJointVelocities();
}

void PxObjectWrapper::set_qvel(const std::vector<physx::PxReal> &v) {}

std::vector<physx::PxReal> PxObjectWrapper::get_qacc() const {
  return std::vector<physx::PxReal>();
}
void PxObjectWrapper::set_qacc(const std::vector<physx::PxReal> &v) {}

std::vector<physx::PxReal> PxObjectWrapper::get_qf() const {
  return std::vector<physx::PxReal>();
}
void PxObjectWrapper::set_qf(const std::vector<physx::PxReal> &v) {}
