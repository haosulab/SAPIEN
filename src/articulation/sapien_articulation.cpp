#include "sapien_articulation.h"
#include "sapien_joint.h"
#include "sapien_link.h"
#include <numeric>
#include <spdlog/spdlog.h>

#define CHECK_SIZE(v)                                                                             \
  {                                                                                               \
    if ((v).size() != dof()) {                                                                    \
      spdlog::error("Input vector size does not match DOF of articulation");                      \
      return;                                                                                     \
    }                                                                                             \
  }

namespace sapien {

std::vector<SLinkBase *> SArticulation::getBaseLinks() {
  std::vector<SLinkBase *> result;
  result.reserve(mLinks.size());
  for (auto &link : mLinks) {
    result.push_back(link.get());
  }
  return result;
}

std::vector<SJointBase *> SArticulation::getBaseJoints() {
  std::vector<SJointBase *> result;
  result.reserve(mJoints.size());
  for (auto &joint : mJoints) {
    result.push_back(joint.get());
  }
  return result;
}

EArticulationType SArticulation::getType() const { return DYNAMIC; }

uint32_t SArticulation::dof() const { return mPxArticulation->getDofs(); }

std::vector<physx::PxReal> SArticulation::getQpos() const {
  mPxArticulation->copyInternalStateToCache(*mCache, PxArticulationCache::ePOSITION);
  return I2E(std::vector<physx::PxReal>(mCache->jointPosition, mCache->jointPosition + dof()));
}

void SArticulation::setQpos(std::vector<physx::PxReal> const &v) {
  CHECK_SIZE(v);

  auto v2 = E2I(v);
  for (size_t i = 0; i < v.size(); ++i) {
    mCache->jointPosition[i] = v2[i];
  }
  mPxArticulation->applyCache(*mCache, PxArticulationCache::ePOSITION);
}

std::vector<physx::PxReal> SArticulation::getQvel() const {
  mPxArticulation->copyInternalStateToCache(*mCache, PxArticulationCache::eVELOCITY);
  return I2E(std::vector<physx::PxReal>(mCache->jointVelocity, mCache->jointVelocity + dof()));
}
void SArticulation::setQvel(std::vector<physx::PxReal> const &v) {
  CHECK_SIZE(v);

  auto v2 = E2I(v);
  for (size_t i = 0; i < v.size(); ++i) {
    mCache->jointVelocity[i] = v2[i];
  }
  mPxArticulation->applyCache(*mCache, PxArticulationCache::eVELOCITY);
}

std::vector<physx::PxReal> SArticulation::getQacc() const {
  mPxArticulation->copyInternalStateToCache(*mCache, PxArticulationCache::eACCELERATION);
  return I2E(
      std::vector<physx::PxReal>(mCache->jointAcceleration, mCache->jointAcceleration + dof()));
}

void SArticulation::setQacc(std::vector<physx::PxReal> const &v) {
  CHECK_SIZE(v);

  auto v2 = E2I(v);
  for (size_t i = 0; i < v.size(); ++i) {
    mCache->jointAcceleration[i] = v2[i];
  }
  mPxArticulation->applyCache(*mCache, PxArticulationCache::eACCELERATION);
}

std::vector<physx::PxReal> SArticulation::getQf() const {
  mPxArticulation->copyInternalStateToCache(*mCache, PxArticulationCache::eFORCE);
  return I2E(std::vector<physx::PxReal>(mCache->jointForce, mCache->jointForce + dof()));
}
void SArticulation::setQf(std::vector<physx::PxReal> const &v) {
  CHECK_SIZE(v);

  auto v2 = E2I(v);
  for (size_t i = 0; i < v.size(); ++i) {
    mCache->jointForce[i] = v2[i];
  }
  mPxArticulation->applyCache(*mCache, PxArticulationCache::eFORCE);
}

std::vector<std::array<physx::PxReal, 2>> SArticulation::getQlimits() const {
  std::vector<std::array<physx::PxReal, 2>> output;
  for (auto &j : mJoints) {
    auto limits = j->getLimits();
    for (auto &l : limits) {
      output.push_back(l);
    }
  }
  return output;
}

// FIXME needs testing
void SArticulation::setQlimits(std::vector<std::array<physx::PxReal, 2>> const &v) const {
  CHECK_SIZE(v);
  uint32_t n = 0;
  for (auto &j : mJoints) {
    uint32_t dof = j->getDof();
    j->setLimits(std::vector<std::array<physx::PxReal, 2>>(v.begin() + n, v.begin() + dof));
    n += dof;
  }
}

// FIXME needs testing
void SArticulation::setDriveTarget(std::vector<physx::PxReal> const &v) {
  CHECK_SIZE(v);

  uint32_t i = 0;
  for (auto &j : mJoints) {
    for (auto axis : j->getAxes()) {
      j->getPxJoint()->setDriveTarget(axis, v[i]);
      i += 1;
    }
  }
}

void SArticulation::setRootPose(physx::PxTransform const &T) {
  mPxArticulation->teleportRootLink(T, true);
}

SArticulation::SArticulation(SScene *scene) : mScene(scene) {}

std::vector<PxReal> SArticulation::E2I(std::vector<PxReal> ev) const {
  std::vector<PxReal> iv(ev.size());
  for (uint32_t i = 0; i < ev.size(); ++i) {
    iv[i] = ev[mIndexI2E[i]];
  }
  return iv;
}

std::vector<PxReal> SArticulation::I2E(std::vector<PxReal> iv) const {
  std::vector<PxReal> ev(iv.size());
  for (uint32_t i = 0; i < iv.size(); ++i) {
    ev[i] = iv[mIndexE2I[i]];
  }
  return ev;
}
std::vector<PxReal> SArticulation::getDriveTarget() const {
  std::vector<PxReal> driveTarget;
  for (auto &j : mJoints) {
    for (auto axis : j->getAxes()) {
      driveTarget.push_back(j->getPxJoint()->getDriveTarget(axis));
    }
  }
  return driveTarget;
}
std::vector<SJoint *> SArticulation::getSJoints() {
  std::vector<SJoint *> result;
  result.reserve(mJoints.size());
  for (auto &joint : mJoints) {
    result.push_back(joint.get());
  }
  return result;
}
} // namespace sapien
