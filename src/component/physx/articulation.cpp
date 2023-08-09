#include "sapien/component/physx/articulation.h"
#include "sapien/component/physx/articulation_link_component.h"
#include "sapien/component/physx/physx_system.h"
#include "sapien/math/conversion.h"
#include "sapien/scene.h"

namespace sapien {
namespace component {
using namespace physx;

static std::shared_ptr<PhysxArticulationLinkComponent>
shared_link(PhysxArticulationLinkComponent *link) {
  return std::static_pointer_cast<PhysxArticulationLinkComponent>(link->shared_from_this());
}

PhysxArticulation::PhysxArticulation() {
  mPxArticulation = PhysxEngine::Get()->getPxPhysics()->createArticulationReducedCoordinate();
  mPxArticulation->setArticulationFlag(PxArticulationFlag::eDRIVE_LIMITS_ARE_FORCES, true);
}

// add link must only be called when PxArticulation is not added to scene
void PhysxArticulation::addLink(PhysxArticulationLinkComponent &link,
                                PhysxArticulationLinkComponent *parent) {
  mLinks.push_back(&link);
  auto pxlink = mPxArticulation->createLink(parent ? parent->getPxActor() : nullptr,
                                            PxTransform{PxIdentity});
  link.internalSetPxLink(pxlink);
  link.internalSetIndex(mLinks.size() - 1);
}

void PhysxArticulation::internalAddLinkAtIndex(PhysxArticulationLinkComponent &link,
                                               PhysxArticulationLinkComponent *parent,
                                               uint32_t index) {
  if (index >= mLinks.size()) {
    throw std::runtime_error("corrupted articulation index");
  }
  mLinks[index] = &link;
  auto pxlink = mPxArticulation->createLink(parent ? parent->getPxActor() : nullptr,
                                            PxTransform{PxIdentity});
  link.internalSetPxLink(pxlink);
  link.internalSetIndex(index);
}

// remove link must only be called when PxArticulation is not added to scene
void PhysxArticulation::removeLink(PhysxArticulationLinkComponent &link) {
  if (link.getChildren().size()) {
    throw std::runtime_error("failed to remove link: only leaf link can be removed");
  }
  if (std::erase(mLinks, &link) != 1) {
    throw std::runtime_error("failed to remove link: link does not exist");
  }
  link.getPxActor()->release();
  link.internalSetPxLink(nullptr);
  refreshLinkIndices();
}

void PhysxArticulation::refreshLinkIndices() {
  for (uint32_t i = 0; i < mLinks.size(); ++i) {
    mLinks[i]->internalSetIndex(i);
  }
}

void PhysxArticulation::syncPose() {
  for (auto link : mLinks) {

    if (!link) {
      continue;
    }

    if (auto entity = link->getEntity()) {
      entity->internalSyncPose(PxTransformToPose(link->getPxActor()->getGlobalPose()));
    }
  }
}

std::vector<std::shared_ptr<PhysxArticulationLinkComponent>>
PhysxArticulation::findDescendants(std::shared_ptr<PhysxArticulationLinkComponent> link) const {
  // optimize?
  std::vector<std::shared_ptr<PhysxArticulationLinkComponent>> links = {link};
  for (auto l : mLinks) {
    if (std::find(links.begin(), links.end(), l->getParent()) != links.end()) {
      links.push_back(shared_link(l));
    }
  }
  return links;
}

void PhysxArticulation::internalEnsureAddedToScene() {
  mScene = nullptr;
  mLinksAddedToScene = 0;
  for (auto l : mLinks) {
    auto scene = l->getScene();
    if (!mScene) {
      mScene = scene.get();
    } else if (mScene != scene.get()) {
      throw std::runtime_error(
          "links of the same articulation cannot be added to different scenes");
    }
    if (scene) {
      mLinksAddedToScene++;
    }
  }
  if (!mPxArticulation->getScene() && mPxArticulation->getNbLinks() == mLinksAddedToScene) {
    internalAddPxArticulationToScene(*mScene);
  }
}

void PhysxArticulation::internalNotifyAddToScene(PhysxArticulationLinkComponent *link,
                                                 Scene &scene) {
  if (mScene && mScene != &scene) {
    throw std::runtime_error("links of the same articulation cannot be added to different scenes");
  }
  mScene = &scene;
  mLinksAddedToScene++;
  if (mPxArticulation->getNbLinks() == mLinksAddedToScene) {
    internalAddPxArticulationToScene(scene);
  }
}

void PhysxArticulation::internalNotifyRemoveFromScene(PhysxArticulationLinkComponent *link,
                                                      Scene &scene) {
  mLinksAddedToScene--;
  internalEnsureRemovedFromScene();
  if (mLinksAddedToScene == 0) {
    mScene = nullptr;
  }
}

void PhysxArticulation::internalAddPxArticulationToScene(Scene &scene) {
  auto system = scene.getPhysxSystem();
  system->getPxScene()->addArticulation(*mPxArticulation);
  mCache = mPxArticulation->createCache();
  updatePermutationMatrix();
  mPxArticulation->setSolverIterationCounts(system->getSceneConfig().solverIterations,
                                            system->getSceneConfig().solverVelocityIterations);
  mPxArticulation->setSleepThreshold(system->getSceneConfig().sleepThreshold);
}

void PhysxArticulation::internalEnsureRemovedFromScene() {
  if (mCache) {
    mCache->release();
    mCache = nullptr;
  }
  if (mPxArticulation->getScene()) {
    mPxArticulation->getScene()->removeArticulation(*mPxArticulation);
  }
}

void PhysxArticulation::updatePermutationMatrix() {
  std::vector<uint32_t> dofStarts(mLinks.size());
  dofStarts[0] = 0;
  for (auto &link : mLinks) {
    auto pxLink = link->getPxActor();
    auto idx = pxLink->getLinkIndex();
    if (idx) {
      dofStarts[idx] = pxLink->getInboundJointDof();
    }
  }
  uint32_t count = 0;
  for (uint32_t i = 1; i < mLinks.size(); ++i) {
    uint32_t dofs = dofStarts[i];
    dofStarts[i] = count;
    count += dofs;
  }
  std::vector<int> jointE2I;
  count = 0;
  for (uint32_t i = 0; i < mLinks.size(); ++i) {
    uint32_t dof = mLinks[i]->getPxActor()->getInboundJointDof();
    uint32_t start = dofStarts[mLinks[i]->getPxActor()->getLinkIndex()];
    for (uint32_t d = 0; d < dof; ++d) {
      jointE2I.push_back(start + d);
    }
  }

  std::vector<int> rowE2I(6 * (mLinks.size() - 1));
  for (size_t k = 1; k < mLinks.size(); ++k) {
    auto internalIndex = mLinks[k]->getPxActor()->getLinkIndex() - 1;
    auto externalIndex = k - 1;
    for (int j = 0; j < 6; ++j) {
      rowE2I[6 * externalIndex + j] = 6 * internalIndex + j;
    }
  }
  mPermutationE2I = Eigen::PermutationMatrix<Eigen::Dynamic>(
      Eigen::Map<Eigen::VectorXi>(jointE2I.data(), jointE2I.size()));
  mLinkPermutationE2I = Eigen::PermutationMatrix<Eigen::Dynamic>(
      Eigen::Map<Eigen::VectorXi>(rowE2I.data(), rowE2I.size()));
}

std::shared_ptr<PhysxArticulationLinkComponent> PhysxArticulation::getRoot() const {
  if (mLinks.size() == 0) {
    throw std::runtime_error("articulation is invalid");
  }
  auto l = mLinks.at(0);
  return shared_link(l);
}

std::vector<std::shared_ptr<PhysxArticulationLinkComponent>> PhysxArticulation::getLinks() const {
  std::vector<std::shared_ptr<PhysxArticulationLinkComponent>> result;
  for (auto l : mLinks) {
    result.push_back(shared_link(l));
  }
  return result;
}

std::vector<std::shared_ptr<PhysxArticulationJoint>> PhysxArticulation::getJoints() const {
  std::vector<std::shared_ptr<PhysxArticulationJoint>> result;
  for (auto l : mLinks) {
    result.push_back(l->getJoint());
  }
  return result;
}

std::vector<std::shared_ptr<PhysxArticulationJoint>> PhysxArticulation::getActiveJoints() const {
  std::vector<std::shared_ptr<PhysxArticulationJoint>> result;
  for (auto l : mLinks) {
    if (l->getJoint()->getDof()) {
      result.push_back(l->getJoint());
    }
  }
  return result;
}

uint32_t PhysxArticulation::getDof() {
  if (!mPxArticulation->getScene()) {
    throw std::runtime_error(
        "articulation properties are only available when all links are added to scene");
  }
  return mPxArticulation->getDofs();
}

void PhysxArticulation::checkDof(uint32_t n) {
  if (n != getDof()) {
    throw std::runtime_error("input vector size does not match articulation DOFs");
  }
}

Eigen::VectorXf PhysxArticulation::getQpos() {
  uint32_t dof = getDof();
  mPxArticulation->copyInternalStateToCache(*mCache, PxArticulationCacheFlag::ePOSITION);
  return mPermutationE2I.inverse() * Eigen::Map<Eigen::VectorXf>(mCache->jointPosition, dof);
}
Eigen::VectorXf PhysxArticulation::getQvel() {
  uint32_t dof = getDof();
  mPxArticulation->copyInternalStateToCache(*mCache, PxArticulationCacheFlag::eVELOCITY);
  return mPermutationE2I.inverse() * Eigen::Map<Eigen::VectorXf>(mCache->jointVelocity, dof);
}
Eigen::VectorXf PhysxArticulation::getQacc() {
  uint32_t dof = getDof();
  mPxArticulation->copyInternalStateToCache(*mCache, PxArticulationCacheFlag::eACCELERATION);
  return mPermutationE2I.inverse() * Eigen::Map<Eigen::VectorXf>(mCache->jointAcceleration, dof);
}
Eigen::VectorXf PhysxArticulation::getQf() {
  uint32_t dof = getDof();
  mPxArticulation->copyInternalStateToCache(*mCache, PxArticulationCacheFlag::eFORCE);
  return mPermutationE2I.inverse() * Eigen::Map<Eigen::VectorXf>(mCache->jointForce, dof);
}

void PhysxArticulation::setQpos(Eigen::VectorXf const &q) {
  checkDof(q.size());
  uint32_t dof = getDof();
  Eigen::Map<Eigen::VectorXf>(mCache->jointPosition, dof) = mPermutationE2I * q;
  mPxArticulation->applyCache(*mCache, PxArticulationCacheFlag::ePOSITION);
  syncPose();
}
void PhysxArticulation::setQvel(Eigen::VectorXf const &q) {
  checkDof(q.size());
  uint32_t dof = getDof();
  Eigen::Map<Eigen::VectorXf>(mCache->jointVelocity, dof) = mPermutationE2I * q;
  mPxArticulation->applyCache(*mCache, PxArticulationCacheFlag::eVELOCITY);
}
void PhysxArticulation::setQacc(Eigen::VectorXf const &q) {
  checkDof(q.size());
  uint32_t dof = getDof();
  Eigen::Map<Eigen::VectorXf>(mCache->jointAcceleration, dof) = mPermutationE2I * q;
  mPxArticulation->applyCache(*mCache, PxArticulationCacheFlag::eACCELERATION);
}
void PhysxArticulation::setQf(Eigen::VectorXf const &q) {
  checkDof(q.size());
  uint32_t dof = getDof();
  Eigen::Map<Eigen::VectorXf>(mCache->jointForce, dof) = mPermutationE2I * q;
  mPxArticulation->applyCache(*mCache, PxArticulationCacheFlag::eFORCE);
}

Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> PhysxArticulation::getQLimit() {
  Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> m;
  m.resize(getDof(), 2);
  uint32_t r = 0;
  for (auto link : mLinks) {
    auto limit = link->getJoint()->getLimit();
    for (auto row : limit.rowwise()) {
      m.row(r++) = row;
    }
  }
  assert(r == getDof());
  return m;
}

Pose PhysxArticulation::getRootPose() {
  return PxTransformToPose(mPxArticulation->getRootGlobalPose());
}
Vec3 PhysxArticulation::getRootLinearVelocity() {
  return PxVec3ToVec3(mPxArticulation->getRootLinearVelocity());
}
Vec3 PhysxArticulation::getRootAngularVelocity() {
  return PxVec3ToVec3(mPxArticulation->getRootAngularVelocity());
}

void PhysxArticulation::setRootPose(Pose const &pose) {
  mPxArticulation->setRootGlobalPose(PoseToPxTransform(pose));
  syncPose();
}
void PhysxArticulation::setRootLinearVelocity(Vec3 const &v) {
  mPxArticulation->setRootLinearVelocity(Vec3ToPxVec3(v));
}
void PhysxArticulation::setRootAngularVelocity(Vec3 const &v) {
  mPxArticulation->setRootAngularVelocity(Vec3ToPxVec3(v));
}

Eigen::VectorXf PhysxArticulation::computePassiveForce(bool gravity, bool coriolisAndCentrifugal) {
  mPxArticulation->commonInit();
  uint32_t n = getDof();

  Eigen::VectorXf f;
  f.resize(n);
  f.setZero();

  if (coriolisAndCentrifugal) {
    mPxArticulation->copyInternalStateToCache(*mCache, PxArticulationCacheFlag::eVELOCITY);
    mPxArticulation->computeCoriolisAndCentrifugalForce(*mCache);
    f += Eigen::Map<Eigen::VectorXf>(mCache->jointForce, n);
  }
  if (gravity) {
    mPxArticulation->computeGeneralizedGravityForce(*mCache);
    f += Eigen::Map<Eigen::VectorXf>(mCache->jointForce, n);
  }
  return f;
}

PhysxArticulation::~PhysxArticulation() {
  if (mCache) {
    mCache->release();
  }
  if (mPxArticulation->getScene()) {
    mPxArticulation->getScene()->removeArticulation(*mPxArticulation);
  }
  mPxArticulation->release();
}

} // namespace component
} // namespace sapien
