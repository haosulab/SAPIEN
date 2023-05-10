#include "sapien/articulation/sapien_joint.h"
#include "sapien/articulation/sapien_link.h"
#include <spdlog/spdlog.h>

namespace sapien {
using namespace physx;

SJointBase::SJointBase(SLinkBase *parent, SLinkBase *child)
    : mParentLink(parent), mChildLink(child) {}

SJoint::SJoint(SArticulation *articulation, SLink *parent, SLink *child,
               PxArticulationJointReducedCoordinate *pxJoint)
    : SJointBase(parent, child), mArticulation(articulation), mPxJoint(pxJoint) {}

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
    spdlog::get("SAPIEN")->critical("Spherical joint not currently supported");
    throw std::runtime_error("Unsupported joint type");
  case PxArticulationJointType::eUNDEFINED:
    spdlog::get("SAPIEN")->critical("Undefined joint encountered in getDof");
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
    spdlog::get("SAPIEN")->critical("Spherical joint not currently supported");
    throw std::runtime_error("Unsupported joint type");
  case PxArticulationJointType::eUNDEFINED:
    spdlog::get("SAPIEN")->critical("Undefined joint encountered in getLimits");
    throw std::runtime_error("Undefined joint");
  }

  throw std::runtime_error("Reached unreachable code in SJoint::getLimits()");
}

void SJoint::setLimits(std::vector<std::array<physx::PxReal, 2>> const &limits) {
  if (!mPxJoint) {
    return;
  }

  if (limits.size() != getDof()) {
    spdlog::get("SAPIEN")->error(
        "Failed to set joint limits: dimensions of argument does not match joint DOF");
    return;
  }
  switch (mPxJoint->getJointType()) {
  case PxArticulationJointType::eFIX:
    return;
  case PxArticulationJointType::eREVOLUTE:
    if (limits[0][1] == std::numeric_limits<float>::infinity()) {
      mPxJoint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);

      if (mArticulation->getPxArticulation()->getScene()) {
        mArticulation->resetCache();
      }

      return;
    }
    mPxJoint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
    mPxJoint->setLimit(PxArticulationAxis::eTWIST, limits[0][0], limits[0][1]);

    if (mArticulation->getPxArticulation()->getScene()) {
      mArticulation->resetCache();
    }

    return;
  case PxArticulationJointType::ePRISMATIC:
    if (limits[0][1] == std::numeric_limits<float>::infinity()) {
      mPxJoint->setMotion(PxArticulationAxis::eX, PxArticulationMotion::eFREE);

      if (mArticulation->getPxArticulation()->getScene()) {
        mArticulation->resetCache();
      }

      return;
    }
    mPxJoint->setMotion(PxArticulationAxis::eX, PxArticulationMotion::eLIMITED);
    mPxJoint->setLimit(PxArticulationAxis::eX, limits[0][0], limits[0][1]);

    if (mArticulation->getPxArticulation()->getScene()) {
      mArticulation->resetCache();
    }

    return;
  case PxArticulationJointType::eSPHERICAL:
    spdlog::get("SAPIEN")->critical("Spherical joint not currently supported");
    throw std::runtime_error("Unsupported joint type");
  case PxArticulationJointType::eUNDEFINED:
    spdlog::get("SAPIEN")->critical("Undefined joint encountered in setLimits");
    throw std::runtime_error("Undefined joint");
  }

  throw std::runtime_error("Reached unreachable code in SJoint::setLimits()");
}

std::vector<PxArticulationAxis::Enum> SJoint::getAxes() const {
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
    spdlog::get("SAPIEN")->critical("Spherical joint not currently supported");
    throw std::runtime_error("Unsupported joint type");
  case PxArticulationJointType::eUNDEFINED:
    spdlog::get("SAPIEN")->critical("Undefined joint encountered in getAxes");
    throw std::runtime_error("Undefined joint");
  }

  throw std::runtime_error("Reached unreachable code in SJoint::getAxes()");
}

void SJoint::setFriction(PxReal coef) {
  if (mPxJoint) {
    mPxJoint->setFrictionCoefficient(coef);
  }
}

PxTransform SJoint::getGlobalPose() const {
  if (mPxJoint) {
    return mChildLink->getPose() * mPxJoint->getChildPose();
  }
  return mChildLink->getPose();
}

void SJoint::setDriveProperty(PxReal stiffness, PxReal damping, PxReal forceLimit,
                              bool useAcceleration) {
  for (auto axis : getAxes()) {
    mPxJoint->setDrive(axis, stiffness, damping, forceLimit,
                       useAcceleration ? PxArticulationDriveType::eACCELERATION
                                       : PxArticulationDriveType::eFORCE);
  }
}

bool SJoint::getDriveIsAcceleration() const {
  auto ax = getAxes();
  if (ax.size()) {
    PxReal stiffness, damping, maxForce;
    PxArticulationDriveType::Enum type;
    mPxJoint->getDrive(ax[0], stiffness, damping, maxForce, type);
    if (type == PxArticulationDriveType::eACCELERATION) {
      return true;
    }
  }
  return false;
}

void SJoint::setDriveVelocityTarget(std::vector<PxReal> const &v) {
  if (v.size() != getDof()) {
    spdlog::get("SAPIEN")->error(
        "Failed to set drive: dimensions of argument does not match joint DOF");
    return;
  }
  auto axes = getAxes();
  for (uint32_t i = 0, s = v.size(); i < s; ++i) {
    mPxJoint->setDriveVelocity(axes[i], v[i]);
  }
  mArticulation->getPxArticulation()->wakeUp();
}

void SJoint::setDriveVelocityTarget(PxReal v) { setDriveVelocityTarget(std::vector{v}); }

void SJoint::setDriveTarget(std::vector<PxReal> const &p) {
  if (p.size() != getDof()) {
    spdlog::get("SAPIEN")->error(
        "Failed to set drive: dimensions of argument does not match joint DOF");
    return;
  }
  auto axes = getAxes();
  for (uint32_t i = 0, s = p.size(); i < s; ++i) {
    mPxJoint->setDriveTarget(axes[i], p[i]);
  }
  mArticulation->getPxArticulation()->wakeUp();
}
void SJoint::setDriveTarget(PxReal p) { setDriveTarget(std::vector{p}); }

PxTransform SJoint::getParentPose() const {
  if (mPxJoint) {
    return mPxJoint->getParentPose();
  }
  return PxTransform(PxIdentity);
}

PxTransform SJoint::getChildPose() const {
  if (mPxJoint) {
    return mPxJoint->getChildPose();
  }
  return PxTransform(PxIdentity);
}

PxReal SJoint::getFriction() const {
  if (mPxJoint) {
    return mPxJoint->getFrictionCoefficient();
  }
  return 0.f;
}

PxReal SJoint::getDriveStiffness() const {
  auto ax = getAxes();
  if (ax.size()) {
    PxReal stiffness, damping, maxForce;
    PxArticulationDriveType::Enum type;
    mPxJoint->getDrive(ax[0], stiffness, damping, maxForce, type);
    return stiffness;
  }
  return 0.f;
}
PxReal SJoint::getDriveDamping() const {
  auto ax = getAxes();
  if (ax.size()) {
    PxReal stiffness, damping, maxForce;
    PxArticulationDriveType::Enum type;
    mPxJoint->getDrive(ax[0], stiffness, damping, maxForce, type);
    return damping;
  }
  return 0.f;
}
PxReal SJoint::getDriveForceLimit() const {
  auto ax = getAxes();
  if (ax.size()) {
    PxReal stiffness, damping, maxForce;
    PxArticulationDriveType::Enum type;
    mPxJoint->getDrive(ax[0], stiffness, damping, maxForce, type);
    return maxForce;
  }
  return 0.f;
}

PxReal SJoint::getDriveTarget() const {
  auto ax = getAxes();
  if (ax.size() > 1) {
    throw std::runtime_error("get drive target not implemented for multi-dof joints");
  }
  if (ax.size()) {
    return mPxJoint->getDriveTarget(ax[0]);
  }
  return 0.f;
}

PxReal SJoint::getDriveVelocityTarget() const {
  auto ax = getAxes();
  if (ax.size() > 1) {
    throw std::runtime_error("get drive target not implemented for multi-dof joints");
  }
  if (ax.size()) {
    return mPxJoint->getDriveVelocity(ax[0]);
  }
  return 0.f;
}

PxArticulationJointType::Enum SJoint::getType() const {
  if (mPxJoint) {
    return mPxJoint->getJointType();
  }
  return PxArticulationJointType::eUNDEFINED;
}

SArticulationBase *SJoint::getArticulation() const { return mArticulation; }

} // namespace sapien
