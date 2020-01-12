#include "sapien_joint.h"
#include "sapien_link.h"
#include <spdlog/spdlog.h>

namespace sapien {
using namespace physx;

SJointBase::SJointBase(SLinkBase *parent, SLinkBase *child)
    : mParentLink(parent), mChildLink(child) {}

SJoint::SJoint(SLink *parent, SLink *child, PxArticulationJointReducedCoordinate *pxJoint)
    : SJointBase(parent, child), mPxJoint(pxJoint) {}

PxArticulationJointReducedCoordinate *SJoint::getPxJoint() { return mPxJoint; }

uint32_t SJoint::getDof() const {
  if (!mPxJoint) {
    return 0;
  }
  switch (mPxJoint->getJointType()) {
  case PxArticulationJointType::eFIX:
    return 0;
  case PxArticulationJointType::eREVOLUTE:
    return 1;
  case PxArticulationJointType::ePRISMATIC:
    return 1;
  case PxArticulationJointType::eSPHERICAL:
    spdlog::critical("Spherical joint not currently supported");
    throw std::runtime_error("Unsupported joint type");
  case PxArticulationJointType::eUNDEFINED:
    spdlog::critical("Undefined joint encountered in getDof");
    throw std::runtime_error("Undefined joint");
  }
  throw std::runtime_error("Reached unreachable code in SJoint::getDof()");
}

std::vector<std::array<physx::PxReal, 2>> SJoint::getLimits() {
  if (!mPxJoint) {
    return {};
  }

  switch (mPxJoint->getJointType()) {
  case PxArticulationJointType::eFIX:
    return {};
  case PxArticulationJointType::eREVOLUTE:
    if (mPxJoint->getMotion(PxArticulationAxis::eTWIST) == PxArticulationMotion::eFREE) {
      return {{-std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()}};
    } else {
      PxReal low, high;
      mPxJoint->getLimit(PxArticulationAxis::eTWIST, low, high);
      return {{low, high}};
    }
  case PxArticulationJointType::ePRISMATIC:
    if (mPxJoint->getMotion(PxArticulationAxis::eX) == PxArticulationMotion::eFREE) {
      return {{-std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()}};
    } else {
      PxReal low, high;
      mPxJoint->getLimit(PxArticulationAxis::eX, low, high);
      return {{low, high}};
    }
  case PxArticulationJointType::eSPHERICAL:
    spdlog::critical("Spherical joint not currently supported");
    throw std::runtime_error("Unsupported joint type");
  case PxArticulationJointType::eUNDEFINED:
    spdlog::critical("Undefined joint encountered in getLimits");
    throw std::runtime_error("Undefined joint");
  }

  throw std::runtime_error("Reached unreachable code in SJoint::getLimits()");
}

void SJoint::setLimits(std::vector<std::array<physx::PxReal, 2>> const &limits) {
  if (!mPxJoint) {
    return;
  }

  if (limits.size() != getDof()) {
    spdlog::error("Failed to set joint limits: dimensions of argument does not match joint DOF");
    return;
  }
  switch (mPxJoint->getJointType()) {
  case PxArticulationJointType::eFIX:
    return;
  case PxArticulationJointType::eREVOLUTE:
    if (limits[0][1] == std::numeric_limits<float>::infinity()) {
      mPxJoint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
      return;
    }
    mPxJoint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
    mPxJoint->setLimit(PxArticulationAxis::eTWIST, limits[0][0], limits[0][1]);
    return;
  case PxArticulationJointType::ePRISMATIC:
    if (limits[0][1] == std::numeric_limits<float>::infinity()) {
      mPxJoint->setMotion(PxArticulationAxis::eX, PxArticulationMotion::eFREE);
      return;
    }
    mPxJoint->setMotion(PxArticulationAxis::eX, PxArticulationMotion::eLIMITED);
    mPxJoint->setLimit(PxArticulationAxis::eX, limits[0][0], limits[0][1]);
    return;
  case PxArticulationJointType::eSPHERICAL:
    spdlog::critical("Spherical joint not currently supported");
    throw std::runtime_error("Unsupported joint type");
  case PxArticulationJointType::eUNDEFINED:
    spdlog::critical("Undefined joint encountered in setLimits");
    throw std::runtime_error("Undefined joint");
  }

  throw std::runtime_error("Reached unreachable code in SJoint::setLimits()");
}

std::vector<PxArticulationAxis::Enum> SJoint::getAxes() {
  if (!mPxJoint) {
    return {};
  }

  switch (mPxJoint->getJointType()) {
  case PxArticulationJointType::eFIX:
    return {};
  case PxArticulationJointType::eREVOLUTE:
    return {PxArticulationAxis::eTWIST};
  case PxArticulationJointType::ePRISMATIC:
    return {PxArticulationAxis::eX};
  case PxArticulationJointType::eSPHERICAL:
    spdlog::critical("Spherical joint not currently supported");
    throw std::runtime_error("Unsupported joint type");
  case PxArticulationJointType::eUNDEFINED:
    spdlog::critical("Undefined joint encountered in getAxes");
    throw std::runtime_error("Undefined joint");
  }

  throw std::runtime_error("Reached unreachable code in SJoint::getAxes()");
}

void SJoint::setFriction(PxReal coef) { mPxJoint->setFrictionCoefficient(coef); }

PxTransform SJoint::getGlobalPose() const {
  return mChildLink->getPose() * mPxJoint->getChildPose();
}

void SJoint::setDriveProperty(PxReal stiffness, PxReal damping, PxReal forceLimit) {
  if (getDof() != 1) {
    spdlog::error("Failed to set drive property, it only supports joints with 1 DOF");
    return;
  }
  for (auto axis : getAxes()) {
    mPxJoint->setDrive(axis, stiffness, damping, forceLimit);
  }
}

void SJoint::setDriveVelocityTarget(std::vector<PxReal> const &v) {
  if (v.size() != getDof()) {
    spdlog::error("Failed to set drive: dimensions of argument does not match joint DOF");
    return;
  }
  auto axes = getAxes();
  for (uint32_t i = 0, s = v.size(); i < s; ++i) {
    mPxJoint->setDriveVelocity(axes[i], v[i]);
  }
}

void SJoint::setDriveVelocityTarget(PxReal v) { setDriveVelocityTarget(std::vector{v}); }

void SJoint::setDriveTarget(std::vector<PxReal> const &p) {
  if (p.size() != getDof()) {
    spdlog::error("Failed to set drive: dimensions of argument does not match joint DOF");
    return;
  }
  auto axes = getAxes();
  for (uint32_t i = 0, s = p.size(); i < s; ++i) {
    mPxJoint->setDriveTarget(axes[i], p[i]);
  }
}
void SJoint::setDriveTarget(PxReal p) { setDriveTarget(std::vector{p}); }

} // namespace sapien
