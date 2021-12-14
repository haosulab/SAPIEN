#include "sapien_articulation.h"
#include "sapien_joint.h"
#include "sapien_link.h"
#include "sapien_scene.h"
#include <numeric>
#include <spdlog/spdlog.h>
#include <stdexcept>

#define CHECK_SIZE(v)                                                                             \
  {                                                                                               \
    if ((v).size() != dof()) {                                                                    \
      throw std::runtime_error("Input vector size does not match DOF of articulation");           \
    }                                                                                             \
  }

namespace sapien {

static Eigen::Matrix3f skewSymmetric(Eigen::Vector3f t) {
  Eigen::Matrix3f hatMatrix;
  hatMatrix << 0, -t(2), t(1), t(2), 0, -t(0), -t(1), t(0), 0;
  return hatMatrix;
};

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

void SArticulation::setRootPose(physx::PxTransform const &T) {
  mPxArticulation->teleportRootLink(T, true);
}

void SArticulation::setRootVelocity(physx::PxVec3 const &v) {
  mRootLink->getPxActor()->setLinearVelocity(v);
}

void SArticulation::setRootAngularVelocity(physx::PxVec3 const &omega) {
  mRootLink->getPxActor()->setAngularVelocity(omega);
}

SLinkBase *SArticulation::getRootLink() const { return mRootLink; }

SArticulation::SArticulation(SScene *scene) : SArticulationDrivable(scene) {}

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

Eigen::Matrix<PxReal, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
SArticulation::buildColumnPermutation(const std::vector<uint32_t> &indexI2E) {
  auto size = indexI2E.size();
  Eigen::Matrix<PxReal, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> permutation(size, size);
  permutation.block(0, 0, size, size) = Eigen::MatrixXf::Constant(size, size, 0);
  for (size_t i = 0; i < size; ++i) {
    permutation(i, indexI2E[i]) = 1;
  }
  return permutation;
}

Eigen::Matrix<PxReal, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
SArticulation::buildRowPermutation() {
  uint32_t nValidLinks = getPxArticulation()->getNbLinks() - 1;
  uint32_t size = nValidLinks * 6;
  Eigen::Matrix<PxReal, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> permutation(size, size);
  permutation.block(0, 0, size, size) = Eigen::MatrixXf::Constant(size, size, 0);

  // Find the index of root link for external order (SLinks), normally it is 0
  uint32_t rootExternalIndex = -1;
  for (auto &link : getSLinks()) {
    auto internalIndex = link->getPxActor()->getLinkIndex();
    if (internalIndex == 0) {
      rootExternalIndex = link->getIndex();
      break;
    }
  }
  assert(rootExternalIndex >= 0);

  for (size_t k = 0; k < nValidLinks + 1; ++k) {
    if (k == rootExternalIndex)
      continue;
    auto internalIndex = getSLinks()[k]->getPxActor()->getLinkIndex() - 1;
    auto externalIndex = k < rootExternalIndex ? k : k - 1;
    for (int j = 0; j < 6; ++j) {
      permutation(externalIndex * 6 + j, internalIndex * 6 + j) = 1;
    }
  }

  return permutation;
}

void SArticulation::setDriveTarget(std::vector<physx::PxReal> const &v) {
  CHECK_SIZE(v);

  uint32_t i = 0;
  for (auto &j : mJoints) {
    if (j->getAxes().size() == 1) {
      j->setDriveTarget(v[i]);
      i += 1;
    }
  }
  mPxArticulation->wakeUp();
}

std::vector<PxReal> SArticulation::getDriveTarget() const {
  std::vector<PxReal> driveTarget;
  for (auto &j : mJoints) {
    if (j->getAxes().size() == 1) {
      driveTarget.push_back(j->getDriveTarget());
    }
  }
  return driveTarget;
}

void SArticulation::setDriveVelocityTarget(std::vector<physx::PxReal> const &v) {
  CHECK_SIZE(v);

  uint32_t i = 0;
  for (auto &j : mJoints) {
    if (j->getAxes().size() == 1) {
      j->setDriveVelocityTarget(v[i]);
      i += 1;
    }
  }
  mPxArticulation->wakeUp();
}

std::vector<PxReal> SArticulation::getDriveVelocityTarget() const {
  std::vector<PxReal> driveTarget;
  for (auto &j : mJoints) {
    if (j->getAxes().size() == 1) {
      driveTarget.push_back(j->getDriveVelocityTarget());
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

std::vector<SJoint *> SArticulation::getActiveJoints() {
  std::vector<SJoint *> activeJoints = {};
  for (auto &j : mJoints) {
    for (size_t i = 0; i < j->getDof(); ++i) {
      activeJoints.push_back(j.get());
    }
  }
  return activeJoints;
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

std::vector<physx::PxReal>
SArticulation::computeGeneralizedExternalForce(std::vector<PxVec3> const &force,
                                               std::vector<PxVec3> const &torque) {
  if (mPxArticulation->getNbLinks() != force.size() ||
      mPxArticulation->getNbLinks() != torque.size()) {
    throw std::runtime_error("Input force and torque does not match number of links.");
  }
  mPxArticulation->commonInit();
  for (uint32_t i = 0; i < force.size(); ++i) {
    mCache->externalForces[i].force = force[i];
    mCache->externalForces[i].torque = torque[i];
  }
  mPxArticulation->computeGeneralizedExternalForce(*mCache);
  std::vector<physx::PxReal> externalForce(mCache->jointForce, mCache->jointForce + dof());
  return I2E(externalForce);
}

std::vector<physx::PxReal> SArticulation::computeForwardDynamics(const std::vector<PxReal> &qf) {
  if (qf.size() != dof()) {
    throw std::runtime_error("Input vector size does not match DOF of articulation");
  }

  std::vector<PxReal> internalQf = E2I(qf);
  mPxArticulation->commonInit();
  mPxArticulation->copyInternalStateToCache(*mCache, PxArticulationCache::eVELOCITY);
  mPxArticulation->copyInternalStateToCache(*mCache, PxArticulationCache::ePOSITION);
  for (size_t i = 0; i < dof(); ++i) {
    mCache->jointForce[i] = internalQf[i];
  }
  mPxArticulation->computeJointAcceleration(*mCache);
  std::vector<physx::PxReal> result(mCache->jointAcceleration, mCache->jointAcceleration + dof());
  return I2E(result);
}

std::vector<physx::PxReal> SArticulation::computeInverseDynamics(const std::vector<PxReal> &qacc) {
  if (qacc.size() != dof()) {
    throw std::runtime_error("Input vector size does not match DOF of articulation");
  }

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

Eigen::Matrix<PxReal, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
SArticulation::computeManipulatorInertiaMatrix() {
  using namespace Eigen;
  mPxArticulation->commonInit();
  mPxArticulation->computeGeneralizedMassMatrix(*mCache);

  int mDof = dof();

  Matrix<PxReal, Dynamic, Dynamic, Eigen::RowMajor> originMass =
      Map<Matrix<PxReal, Dynamic, Dynamic, Eigen::RowMajor>>(mCache->massMatrix, mDof, mDof);

  return mColumnPermutationI2E.transpose() * originMass * mColumnPermutationI2E;
}

void SArticulation::prestep() {
  auto time = mParentScene->getTimestep();
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

Eigen::Matrix<PxReal, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
SArticulation::computeSpatialTwistJacobianMatrix() {
  // NOTE: 1. PhysX computeDenseJacobian computes Jacobian for the 6D root link
  // motion, which we discard. 2. PhysX computes the Jacobian for Cartesian
  // velocity, for twist Jacobian, see computeSpatialTwistJacobianMatrix.
  using namespace Eigen;
  PxU32 nRows;
  PxU32 nCols;
  mPxArticulation->computeDenseJacobian(*mCache, nRows, nCols);
  uint32_t freeBase = (nCols == dof()) ? 0 : 6;
  Matrix<PxReal, Dynamic, Dynamic, Eigen::RowMajor> originJacobian =
      Map<Matrix<PxReal, Dynamic, Dynamic, Eigen::RowMajor>>(mCache->denseJacobian, nRows, nCols);
  Matrix<PxReal, Dynamic, Dynamic, Eigen::RowMajor> eliminatedJacobian(
      originJacobian.block(freeBase, freeBase, nRows - freeBase, nCols - freeBase));

  std::vector<PxArticulationLink *> internalLinks(mPxArticulation->getNbLinks());
  mPxArticulation->getLinks(internalLinks.data(), mPxArticulation->getNbLinks());

  Matrix<PxReal, Dynamic, Dynamic, Eigen::RowMajor> vel2twist =
      Matrix<PxReal, Dynamic, Dynamic, Eigen::RowMajor>::Identity(nRows - freeBase,
                                                                  nRows - freeBase);
  for (size_t i = 1; i < internalLinks.size(); ++i) {
    auto p = internalLinks[i]->getGlobalPose().p;
    vel2twist.block<3, 3>(6 * i - 6, 6 * i - 3) = skewSymmetric({p[0], p[1], p[2]});
  }

  eliminatedJacobian = vel2twist * eliminatedJacobian;

  // Switch joint(column) order from internal to external
  eliminatedJacobian = eliminatedJacobian * mColumnPermutationI2E;

  // Switch link(row) order from internal to external
  return mRowPermutationI2E * eliminatedJacobian;
}

Eigen::Matrix<PxReal, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
SArticulation::computeWorldCartesianJacobianMatrix() {
  // NOTE: this function computes the Jacobian for twist motion, commonly used
  // in robotics.
  using namespace Eigen;
  PxU32 nRows;
  PxU32 nCols;
  mPxArticulation->computeDenseJacobian(*mCache, nRows, nCols);
  uint32_t freeBase = (nCols == dof()) ? 0 : 6;
  Matrix<PxReal, Dynamic, Dynamic, Eigen::RowMajor> originJacobian =
      Map<Matrix<PxReal, Dynamic, Dynamic, Eigen::RowMajor>>(mCache->denseJacobian, nRows, nCols);
  Matrix<PxReal, Dynamic, Dynamic, Eigen::RowMajor> eliminatedJacobian(
      originJacobian.block(freeBase, freeBase, nRows - freeBase, nCols - freeBase));

  // Switch joint(column) order from internal to external
  eliminatedJacobian = eliminatedJacobian * mColumnPermutationI2E;

  // Switch link(row) order from internal to external
  return mRowPermutationI2E * eliminatedJacobian;
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
    throw std::runtime_error(
        "Failed to unpack articulation data: " + std::to_string(ndof * 4 + nlinks * 12 + 19) +
        " numbers expected but " + std::to_string(data.size()) + " provided");
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

std::vector<PxReal> SArticulation::packDrive() {
  std::vector<PxReal> data;
  std::vector<PxReal> driveTarget;
  std::vector<PxReal> driveVel;
  std::vector<PxReal> driveP;
  std::vector<PxReal> driveD;
  std::vector<PxReal> driveMaxForce;
  uint32_t i = 0;
  for (auto &j : mJoints) {
    for (auto axis : j->getAxes()) {
      driveTarget.push_back(j->getPxJoint()->getDriveTarget(axis));
      driveVel.push_back(j->getPxJoint()->getDriveVelocity(axis));
      PxReal stiffness, damping, maxForce;
      PxArticulationDriveType::Enum driveType;
      j->getPxJoint()->getDrive(axis, stiffness, damping, maxForce, driveType);
      driveP.push_back(stiffness);
      driveD.push_back(damping);
      driveMaxForce.push_back(maxForce);
      i += 1;
    }
  }
  data.insert(data.end(), driveTarget.begin(), driveTarget.end());
  data.insert(data.end(), driveVel.begin(), driveVel.end());
  data.insert(data.end(), driveP.begin(), driveP.end());
  data.insert(data.end(), driveD.begin(), driveD.end());
  data.insert(data.end(), driveMaxForce.begin(), driveMaxForce.end());
  return data;
}

void SArticulation::unpackDrive(std::vector<PxReal> const &data) {
  if (data.size() != dof() * 5) {
    throw std::runtime_error("Invalid data passed to unpackDrive");
  }
  auto it = data.begin();
  std::vector<PxReal> driveTarget(it, it + dof());
  it += dof();
  std::vector<PxReal> driveVel(it, it + dof());
  it += dof();
  std::vector<PxReal> driveP(it, it + dof());
  it += dof();
  std::vector<PxReal> driveD(it, it + dof());
  it += dof();
  std::vector<PxReal> driveMaxForce(it, it + dof());

  uint32_t i = 0;
  for (auto &j : mJoints) {
    for (auto axis : j->getAxes()) {
      j->getPxJoint()->setDriveTarget(axis, driveTarget[i]);
      j->getPxJoint()->setDriveVelocity(axis, driveVel[i]);
      j->getPxJoint()->setDrive(axis, driveP[i], driveD[i], driveMaxForce[i]);
      i += 1;
    }
  }
}

Matrix<PxReal, Dynamic, 1>
SArticulation::computeTwistDiffIK(const Eigen::Matrix<PxReal, 6, 1> &spatialTwist,
                                  uint32_t commandedLinkId,
                                  const std::vector<uint32_t> &activeQIds) {
  auto logger = spdlog::get("SAPIEN");
  auto numCol = activeQIds.empty() ? dof() : activeQIds.size();
  Eigen::VectorXf qvel(numCol);

  if (commandedLinkId == 0) {
    logger->warn("Link with id 0 (root link) can not be a valid commanded link.");
    return qvel;
  }

  auto denseJacobian = computeSpatialTwistJacobianMatrix();
  Eigen::MatrixXf jacobian = denseJacobian.block(commandedLinkId * 6 - 6, 0, 6, dof());
  Eigen::MatrixXf reducedJacobian(jacobian);
  if (!activeQIds.empty()) {
    reducedJacobian.resize(6, numCol);
    for (size_t i = 0; i < numCol; ++i) {
      if (activeQIds[i] >= dof()) {
        logger->warn("Articulation has {} joints, but given joint id {}", dof(), activeQIds[i]);
        return qvel;
      }
      reducedJacobian.block<6, 1>(0, i) = jacobian.block<6, 1>(0, activeQIds[i]);
    }
  }

  Eigen::JacobiSVD<Eigen::MatrixXf> svd_of_j(reducedJacobian,
                                             Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::MatrixXf &u = svd_of_j.matrixU();
  const Eigen::MatrixXf &v = svd_of_j.matrixV();
  const Eigen::VectorXf &s = svd_of_j.singularValues();

  Eigen::VectorXf invS = s;
  static const float epsilon = std::numeric_limits<float>::epsilon();
  double maxS = s[0];
  for (std::size_t i = 0; i < static_cast<std::size_t>(s.rows()); ++i) {
    invS(i) = fabs(s(i)) > maxS * epsilon ? 1.0 / s(i) : 0.0;
  }
  qvel = v * invS.asDiagonal() * u.transpose() * spatialTwist;
  return qvel;
}

Eigen::Matrix<PxReal, 6, 6, Eigen::RowMajor>
SArticulation::computeAdjointMatrix(SLink *sourceFrame, SLink *targetFrame) {
  auto mat44 = computeRelativeTransformation(sourceFrame, targetFrame);
  Eigen::Matrix<PxReal, 6, 6, Eigen::RowMajor> adjoint =
      Eigen::Matrix<PxReal, 6, 6, Eigen::RowMajor>::Zero(6, 6);
  adjoint.block<3, 3>(0, 0) = mat44.block<3, 3>(0, 0);
  adjoint.block<3, 3>(3, 3) = mat44.block<3, 3>(0, 0);
  Eigen::Vector3f position = mat44.block<3, 1>(0, 3);
  adjoint.block<3, 3>(0, 3) = skewSymmetric(position) * mat44.block<3, 3>(0, 0);
  return adjoint;
}

Matrix<PxReal, Dynamic, 1>
SArticulation::computeCartesianVelocityDiffIK(const Eigen::Matrix<PxReal, 6, 1> &cartesianVelocity,
                                              uint32_t commandedLinkId,
                                              const std::vector<uint32_t> &activeQIds) {
  auto logger = spdlog::get("SAPIEN");
  auto numCol = activeQIds.empty() ? dof() : activeQIds.size();
  Eigen::VectorXf qvel(numCol);

  if (commandedLinkId == 0) {
    logger->warn("Link with id 0 (root link) can not be a valid commanded link.");
    return qvel;
  }

  auto denseJacobian = computeWorldCartesianJacobianMatrix();
  Eigen::MatrixXf jacobian = denseJacobian.block(commandedLinkId * 6 - 6, 0, 6, dof());
  Eigen::MatrixXf reducedJacobian(jacobian);
  if (!activeQIds.empty()) {
    reducedJacobian.resize(6, numCol);
    for (size_t i = 0; i < numCol; ++i) {
      if (activeQIds[i] >= dof()) {
        logger->warn("Articulation has {} joints, but given joint id {}", dof(), activeQIds[i]);
        return qvel;
      }
      reducedJacobian.block<6, 1>(0, i) = jacobian.block<6, 1>(0, activeQIds[i]);
    }
  }

  Eigen::JacobiSVD<Eigen::MatrixXf> svd_of_j(reducedJacobian,
                                             Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::MatrixXf &u = svd_of_j.matrixU();
  const Eigen::MatrixXf &v = svd_of_j.matrixV();
  const Eigen::VectorXf &s = svd_of_j.singularValues();

  Eigen::VectorXf invS = s;
  static const float epsilon = std::numeric_limits<float>::epsilon();
  double maxS = s[0];
  for (std::size_t i = 0; i < static_cast<std::size_t>(s.rows()); ++i) {
    invS(i) = fabs(s(i)) > maxS * epsilon ? 1.0 / s(i) : 0.0;
  }
  qvel = v * invS.asDiagonal() * u.transpose() * cartesianVelocity;
  return qvel;
}

Eigen::Matrix<PxReal, 4, 4, Eigen::RowMajor>
SArticulation::computeRelativeTransformation(SLink *sourceFrame, SLink *targetFrame) {
  auto s = sourceFrame->getPose();
  auto t = targetFrame->getPose();
  auto relative = t.getInverse().transform(s);

  Eigen::Matrix<PxReal, 4, 4, Eigen::RowMajor> mat44 =
      Eigen::Matrix<PxReal, 4, 4, Eigen::RowMajor>::Identity(4, 4);
  Eigen::Quaternionf quat(relative.q.w, relative.q.x, relative.q.y, relative.q.z);
  mat44.block<3, 3>(0, 0) = quat.normalized().toRotationMatrix();
  mat44.block<3, 1>(0, 3) = Eigen::Matrix<PxReal, 3, 1>(relative.p.x, relative.p.y, relative.p.z);
  return mat44;
}
Eigen::Matrix<PxReal, 4, 4, Eigen::RowMajor>
SArticulation::computeRelativeTransformation(uint32_t sourceLinkId, uint32_t targetLinkId) {
  auto links = getSLinks();
  return computeRelativeTransformation(links.at(sourceLinkId), links.at(targetLinkId));
}
Eigen::Matrix<PxReal, 6, 6, Eigen::RowMajor>
SArticulation::computeAdjointMatrix(uint32_t sourceLinkId, uint32_t targetLinkId) {
  auto links = getSLinks();
  return computeAdjointMatrix(links.at(sourceLinkId), links.at(targetLinkId));
}

} // namespace sapien

#undef CHECK_SIZE
