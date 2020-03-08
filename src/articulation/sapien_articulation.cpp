#include "sapien_articulation.h"
#include "sapien_joint.h"
#include "sapien_link.h"
#include "sapien_scene.h"
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

EArticulationType SArticulation::getType() const { return EArticulationType::DYNAMIC; }

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

void SArticulation::setRootVelocity(physx::PxVec3 const &v) {
  mRootLink->getPxActor()->setLinearVelocity(v);
}

void SArticulation::setRootAngularVelocity(physx::PxVec3 const &omega) {
  mRootLink->getPxActor()->setAngularVelocity(omega);
}

SLinkBase *SArticulation::getRootLink() { return mRootLink; }

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

std::vector<SLink *> SArticulation::getSLinks() {
  std::vector<SLink *> result;
  result.reserve(mLinks.size());
  for (auto &link : mLinks) {
    result.push_back(link.get());
  }
  return result;
}

std::vector<SJoint *> SArticulation::getSJoints() {
  std::vector<SJoint *> result;
  result.reserve(mJoints.size());
  for (auto &joint : mJoints) {
    result.push_back(joint.get());
  }
  return result;
}

void SArticulation::resetCache() {
  mPxArticulation->releaseCache(*mCache);
  mCache = mPxArticulation->createCache();
}
std::vector<physx::PxReal>
SArticulation::computePassiveForce(bool gravity, bool coriolisAndCentrifugal, bool external) {
  mPxArticulation->commonInit();
  std::vector<physx::PxReal> passiveForce(dof(), 0);

  if (coriolisAndCentrifugal) {
    mPxArticulation->copyInternalStateToCache(*mCache, PxArticulationCache::eVELOCITY);
    mPxArticulation->computeCoriolisAndCentrifugalForce(*mCache);
    std::vector<physx::PxReal> coriolisForce(mCache->jointForce, mCache->jointForce + dof());
    for (size_t j = 0; j < dof(); ++j) {
      passiveForce[j] += coriolisForce[j];
    }
  }

  if (gravity) {
    mPxArticulation->computeGeneralizedGravityForce(*mCache);
    std::vector<physx::PxReal> gravityForce(mCache->jointForce, mCache->jointForce + dof());
    for (size_t j = 0; j < dof(); ++j) {
      passiveForce[j] += gravityForce[j];
    }
  }

  if (external) {
    mPxArticulation->computeGeneralizedExternalForce(*mCache);
    std::vector<physx::PxReal> externalForce(mCache->jointForce, mCache->jointForce + dof());
    for (size_t j = 0; j < dof(); ++j) {
      passiveForce[j] += externalForce[j];
    }
  }

  return I2E(passiveForce);
}

std::vector<physx::PxReal> SArticulation::computeInverseDynamics(const std::vector<PxReal> &qacc) {
  assert(qacc.size() == dof());
  std::vector<PxReal> internalQacc = E2I(qacc);
  mPxArticulation->commonInit();
  mPxArticulation->copyInternalStateToCache(*mCache, PxArticulationCache::eVELOCITY);
  mPxArticulation->copyInternalStateToCache(*mCache, PxArticulationCache::ePOSITION);

  for (size_t i = 0; i < dof(); ++i) {
    mCache->jointAcceleration[i] = internalQacc[i];
  }
  mPxArticulation->computeJointForce(*mCache);
  std::vector<physx::PxReal> result(mCache->jointForce, mCache->jointForce + dof());
  return I2E(result);
}

void SArticulation::prestep() {
  auto time = mScene->getTimestep();
  EventArticulationStep s;
  s.articulation = this;
  s.time = time;
  EventEmitter<EventArticulationStep>::emit(s);

  for (auto &l : mLinks) {
    EventActorStep s;
    s.actor = l.get();
    s.time = time;
    l->EventEmitter<EventActorStep>::emit(s);
  }
}

std::vector<PxReal> SArticulation::computeJacobianMatrix() {
  // TODO: This function is temporary for use without ROS, after ROS2, it will be deleted.
  PxU32 nRows;
  PxU32 nCols;
  mPxArticulation->computeDenseJacobian(*mCache, nRows, nCols);

  std::vector<float> Jacobian(mCache->denseJacobian, mCache->denseJacobian + nRows * nCols);
  std::vector<float> Jacobian2;
  for (size_t j = 0; j < Jacobian.size() + 6; ++j) {
    Jacobian2.push_back(0);
  }
  for (auto &link : mLinks) {
    // result.push_back(link.get());
    auto index = link->getPxActor()->getLinkIndex();
    auto midx = link->getIndex();
    // spdlog::error("Input vector size does not match DOF of articulation {:d} {:d} {:d} {:d}
    // {:d}", index, midx, nRows, nCols, Jacobian2.size());
    if (index == 0)
      continue;
    for (size_t j = 0; j < 6; ++j) {
      for (size_t col = 0; col < nCols; ++col) {
        Jacobian2[(midx * 6 + j) * nCols + mIndexI2E[col]] =
            Jacobian[((index - 1) * 6 + j) * nCols + col];
      }
    }
    midx += 1;
  }
  return Jacobian2;
}

#define PUSH_QUAT(data, q)                                                                        \
  {                                                                                               \
    (data).push_back((q).x);                                                                      \
    (data).push_back((q).y);                                                                      \
    (data).push_back((q).z);                                                                      \
    (data).push_back((q).w);                                                                      \
  }

#define PUSH_VEC3(data, v)                                                                        \
  {                                                                                               \
    (data).push_back((v).x);                                                                      \
    (data).push_back((v).y);                                                                      \
    (data).push_back((v).z);                                                                      \
  }

std::vector<PxReal> SArticulation::packData() {
  std::vector<PxReal> data;

  mPxArticulation->copyInternalStateToCache(*mCache, PxArticulationCache::eALL);
  auto ndof = mPxArticulation->getDofs();
  auto nlinks = mPxArticulation->getNbLinks();

  data.insert(data.end(), mCache->jointPosition, mCache->jointPosition + ndof);
  data.insert(data.end(), mCache->jointVelocity, mCache->jointVelocity + ndof);
  data.insert(data.end(), mCache->jointAcceleration, mCache->jointAcceleration + ndof);
  data.insert(data.end(), mCache->jointForce, mCache->jointForce + ndof);

  for (uint32_t i = 0; i < nlinks; ++i) {
    auto lv = mCache->linkVelocity[i].linear;
    auto av = mCache->linkVelocity[i].angular;
    PUSH_VEC3(data, lv);
    PUSH_VEC3(data, av);
  }

  for (uint32_t i = 0; i < nlinks; ++i) {
    auto la = mCache->linkAcceleration[i].linear;
    auto aa = mCache->linkAcceleration[i].angular;
    PUSH_VEC3(data, la);
    PUSH_VEC3(data, aa);
  }

  auto [transform, lv, av, la, aa] = *mCache->rootLinkData;
  PUSH_VEC3(data, transform.p);
  PUSH_QUAT(data, transform.q);
  PUSH_VEC3(data, lv);
  PUSH_VEC3(data, av);
  PUSH_VEC3(data, la);
  PUSH_VEC3(data, aa);

  return data;
}

void SArticulation::unpackData(std::vector<PxReal> const &data) {
  mPxArticulation->copyInternalStateToCache(*mCache, PxArticulationCache::eALL);
  auto ndof = mPxArticulation->getDofs();
  auto nlinks = mPxArticulation->getNbLinks();

  if (data.size() != ndof * 4          // joint size
                         + nlinks * 12 // link size
                         + 19          // root size
  ) {
    spdlog::error("Failed to unpack articulation data: {} numbers expected but {} provided",
                  ndof * 4 + nlinks * 12 + 19, data.size());
    return;
  }

  mPxArticulation->zeroCache(*mCache);
  uint32_t p = 0;

  // restore joints
  for (uint32_t j = 0; j < ndof; ++j) {
    mCache->jointPosition[j] = data[p++];
  }
  for (uint32_t j = 0; j < ndof; ++j) {
    mCache->jointVelocity[j] = data[p++];
  }
  for (uint32_t j = 0; j < ndof; ++j) {
    mCache->jointAcceleration[j] = data[p++];
  }
  for (uint32_t j = 0; j < ndof; ++j) {
    mCache->jointForce[j] = data[p++];
  }

  // restore links
  for (uint32_t i = 0; i < nlinks; ++i) {
    mCache->linkVelocity[i].linear = {data[p], data[p + 1], data[p + 2]};
    p += 3;
    mCache->linkVelocity[i].angular = {data[p], data[p + 1], data[p + 2]};
    p += 3;
  }
  for (uint32_t i = 0; i < nlinks; ++i) {
    mCache->linkAcceleration[i].linear = {data[p], data[p + 1], data[p + 2]};
    p += 3;
    mCache->linkAcceleration[i].angular = {data[p], data[p + 1], data[p + 2]};
    p += 3;
  }

  // restore root
  mCache->rootLinkData->transform = {{data[p], data[p + 1], data[p + 2]},
                                     {data[p + 3], data[p + 4], data[p + 5], data[p + 6]}};
  p += 7;
  mCache->rootLinkData->worldLinVel = {data[p], data[p + 1], data[p + 2]};
  p += 3;
  mCache->rootLinkData->worldAngVel = {data[p], data[p + 1], data[p + 2]};
  p += 3;
  mCache->rootLinkData->worldLinAccel = {data[p], data[p + 1], data[p + 2]};
  p += 3;
  mCache->rootLinkData->worldAngAccel = {data[p], data[p + 1], data[p + 2]};
  p += 3;

  mPxArticulation->applyCache(*mCache, PxArticulationCache::eALL);
}
// namespace sapien

} // namespace sapien

#undef CHECK_SIZE
