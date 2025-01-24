/*
 * Copyright 2025 Hillbot Inc.
 * Copyright 2020-2024 UCSD SU Lab
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "sapien/physx/articulation.h"
#include "../logger.h"
#include "sapien/math/conversion.h"
#include "sapien/physx/articulation_link_component.h"
#include "sapien/physx/physx_system.h"
#include "sapien/scene.h"

using namespace physx;

namespace sapien {
namespace physx {

static std::shared_ptr<PhysxArticulationLinkComponent>
shared_link(PhysxArticulationLinkComponent *link) {
  return std::static_pointer_cast<PhysxArticulationLinkComponent>(link->shared_from_this());
}

PhysxArticulation::PhysxArticulation() {
  mEngine = PhysxEngine::Get();
  mPxArticulation = PhysxEngine::Get()->getPxPhysics()->createArticulationReducedCoordinate();
  mPxArticulation->setArticulationFlag(PxArticulationFlag::eDRIVE_LIMITS_ARE_FORCES, true);

  mPxArticulation->setSolverIterationCounts(
      PhysxDefault::getBodyConfig().solverPositionIterations,
      PhysxDefault::getBodyConfig().solverVelocityIterations);
  mPxArticulation->setSleepThreshold(PhysxDefault::getBodyConfig().sleepThreshold);
}

void PhysxArticulation::setSolverPositionIterations(uint32_t count) {
  getPxArticulation()->setSolverIterationCounts(count, getSolverVelocityIterations());
}
void PhysxArticulation::setSolverVelocityIterations(uint32_t count) {
  getPxArticulation()->setSolverIterationCounts(getSolverPositionIterations(), count);
}
void PhysxArticulation::setSleepThreshold(float threshold) {
  getPxArticulation()->setSleepThreshold(threshold);
}

uint32_t PhysxArticulation::getSolverPositionIterations() const {
  uint32_t pos, vel;
  getPxArticulation()->getSolverIterationCounts(pos, vel);
  return pos;
}
uint32_t PhysxArticulation::getSolverVelocityIterations() const {
  uint32_t pos, vel;
  getPxArticulation()->getSolverIterationCounts(pos, vel);
  return vel;
}
float PhysxArticulation::getSleepThreshold() const {
  return getPxArticulation()->getSleepThreshold();
}

// add link must only be called when PxArticulation is not added to scene
void PhysxArticulation::addLink(PhysxArticulationLinkComponent &link,
                                PhysxArticulationLinkComponent *parent) {
  mLinks.push_back(&link);
  auto pxlink = mPxArticulation->createLink(parent ? parent->getPxActor() : nullptr,
                                            PxTransform{PxIdentity});
  link.internalSetPxLink(pxlink);
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

#ifdef SAPIEN_CUDA
  if (auto s = std::dynamic_pointer_cast<PhysxSystemGpu>(system)) {
    // Apply GPU scene offset
    Vec3 offset = s->getSceneOffset(scene.shared_from_this());
    PxTransform pose = getPxArticulation()->getRootGlobalPose();
    pose.p = pose.p + PxVec3(offset.x, offset.y, offset.z);
    getPxArticulation()->setRootGlobalPose(pose);
  }
#endif

  system->getPxScene()->addArticulation(*mPxArticulation);
  mCache = mPxArticulation->createCache();
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

std::shared_ptr<PhysxArticulationLinkComponent> PhysxArticulation::getRoot() const {
  if (mLinks.size() == 0) {
    throw std::runtime_error("articulation is invalid");
  }
  auto l = mLinks.at(0);
  return shared_link(l);
}

std::vector<std::shared_ptr<PhysxArticulationLinkComponent>>
PhysxArticulation::getLinksAdditionOrder() const {
  std::vector<std::shared_ptr<PhysxArticulationLinkComponent>> result;
  for (auto l : mLinks) {
    result.push_back(shared_link(l));
  }
  return result;
}

std::vector<std::shared_ptr<PhysxArticulationLinkComponent>> PhysxArticulation::getLinks() const {
  auto result = getLinksAdditionOrder();
  std::sort(result.begin(), result.end(), [](auto const &a, auto const &b) {
    return a->getPxActor()->getLinkIndex() < b->getPxActor()->getLinkIndex();
  });
  return result;
}

std::vector<std::shared_ptr<PhysxArticulationJoint>> PhysxArticulation::getJoints() const {
  std::vector<std::shared_ptr<PhysxArticulationJoint>> result;
  for (auto l : getLinks()) {
    result.push_back(l->getJoint());
  }
  return result;
}

std::vector<std::shared_ptr<PhysxArticulationJoint>> PhysxArticulation::getActiveJoints() const {
  std::vector<std::shared_ptr<PhysxArticulationJoint>> result;
  for (auto l : getLinks()) {
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

  if (getRoot()->isUsingDirectGPUAPI()) {

    static bool once = []() {
      logger::warn("fetching articulation qpos from GPU is very slow and should be avoided");
      return true;
    }();
    (void)once;
#ifdef SAPIEN_CUDA
    auto qpos = std::dynamic_pointer_cast<PhysxSystemGpu>(mScene->getPhysxSystem())
                    ->gpuDownloadArticulationQpos(mPxArticulation->getGpuArticulationIndex());
    return Eigen::Map<Eigen::VectorXf>(qpos.data(), dof);
#else
    return Eigen::VectorXf();
#endif
  }

  mPxArticulation->copyInternalStateToCache(*mCache, PxArticulationCacheFlag::ePOSITION);
  return Eigen::Map<Eigen::VectorXf>(mCache->jointPosition, dof);
}
Eigen::VectorXf PhysxArticulation::getQvel() {
  if (getRoot()->isUsingDirectGPUAPI()) {
    throw std::runtime_error("getting qvel is not supported in GPU simulation.");
  }
  uint32_t dof = getDof();
  mPxArticulation->copyInternalStateToCache(*mCache, PxArticulationCacheFlag::eVELOCITY);
  return Eigen::Map<Eigen::VectorXf>(mCache->jointVelocity, dof);
}
Eigen::VectorXf PhysxArticulation::getQacc() {
  if (getRoot()->isUsingDirectGPUAPI()) {
    throw std::runtime_error("getting qacc is not supported in GPU simulation.");
  }
  uint32_t dof = getDof();
  mPxArticulation->copyInternalStateToCache(*mCache, PxArticulationCacheFlag::eACCELERATION);
  return Eigen::Map<Eigen::VectorXf>(mCache->jointAcceleration, dof);
}
Eigen::VectorXf PhysxArticulation::getQf() {
  if (getRoot()->isUsingDirectGPUAPI()) {
    throw std::runtime_error("getting qf is not supported in GPU simulation.");
  }
  uint32_t dof = getDof();
  mPxArticulation->copyInternalStateToCache(*mCache, PxArticulationCacheFlag::eFORCE);
  return Eigen::Map<Eigen::VectorXf>(mCache->jointForce, dof);
}

void PhysxArticulation::setQpos(Eigen::VectorXf const &q) {
  checkDof(q.size());
  uint32_t dof = getDof();
  if (getRoot()->isUsingDirectGPUAPI()) {

    static bool once = []() {
      logger::warn("setting articulation qpos to GPU is very slow and should be avoided");
      return true;
    }();
    (void)once;
#ifdef SAPIEN_CUDA
    std::dynamic_pointer_cast<PhysxSystemGpu>(mScene->getPhysxSystem())
        ->gpuUploadArticulationQpos(mPxArticulation->getGpuArticulationIndex(), q);
#endif
    return;
  }
  Eigen::Map<Eigen::VectorXf>(mCache->jointPosition, dof) = q;
  mPxArticulation->applyCache(*mCache, PxArticulationCacheFlag::ePOSITION);
  syncPose();
}
void PhysxArticulation::setQvel(Eigen::VectorXf const &q) {
  if (getRoot()->isUsingDirectGPUAPI()) {
    throw std::runtime_error("setting qvel is not supported in GPU simulation.");
  }
  checkDof(q.size());
  uint32_t dof = getDof();
  Eigen::Map<Eigen::VectorXf>(mCache->jointVelocity, dof) = q;
  mPxArticulation->applyCache(*mCache, PxArticulationCacheFlag::eVELOCITY);
}
void PhysxArticulation::setQacc(Eigen::VectorXf const &q) {
  if (getRoot()->isUsingDirectGPUAPI()) {
    throw std::runtime_error("setting qacc is not supported in GPU simulation.");
  }
  checkDof(q.size());
  uint32_t dof = getDof();
  Eigen::Map<Eigen::VectorXf>(mCache->jointAcceleration, dof) = q;
  mPxArticulation->applyCache(*mCache, PxArticulationCacheFlag::eACCELERATION);
}
void PhysxArticulation::setQf(Eigen::VectorXf const &q) {
  if (getRoot()->isUsingDirectGPUAPI()) {
    throw std::runtime_error("setting qf is not supported in GPU simulation.");
  }
  checkDof(q.size());
  uint32_t dof = getDof();
  Eigen::Map<Eigen::VectorXf>(mCache->jointForce, dof) = q;
  mPxArticulation->applyCache(*mCache, PxArticulationCacheFlag::eFORCE);
}

Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> PhysxArticulation::getQLimit() {
  Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> m;
  m.resize(getDof(), 2);
  uint32_t r = 0;
  for (auto link : getLinks()) {
    auto limit = link->getJoint()->getLimit();
    for (auto row : limit.rowwise()) {
      m.row(r++) = row;
    }
  }
  assert(r == getDof());
  return m;
}

Eigen::Matrix<float, Eigen::Dynamic, 6, Eigen::RowMajor>
PhysxArticulation::getLinkIncomingJointForces() {
  if (getRoot()->isUsingDirectGPUAPI()) {
    throw std::runtime_error("getting qf is not supported in GPU simulation.");
  }
  mPxArticulation->copyInternalStateToCache(*mCache,
                                            PxArticulationCacheFlag::eLINK_INCOMING_JOINT_FORCE);
  auto mat8 = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 8, Eigen::RowMajor>>(
      reinterpret_cast<float *>(mCache->linkIncomingJointForce), mPxArticulation->getNbLinks(), 8);
  return mat8(Eigen::all, {0, 1, 2, 4, 5, 6});
}

Pose PhysxArticulation::getRootPose() {
  // if (getRoot()->isUsingDirectGPUAPI()) {
  //   throw std::runtime_error("getting root pose is not supported in GPU simulation.");
  // }
  return PxTransformToPose(mPxArticulation->getRootGlobalPose());
}
Vec3 PhysxArticulation::getRootLinearVelocity() {
  // if (getRoot()->isUsingDirectGPUAPI()) {
  //   throw std::runtime_error("getting root velocity is not supported in GPU simulation.");
  // }
  return PxVec3ToVec3(mPxArticulation->getRootLinearVelocity());
}
Vec3 PhysxArticulation::getRootAngularVelocity() {
  // if (getRoot()->isUsingDirectGPUAPI()) {
  //   throw std::runtime_error("getting root velocity is not supported in GPU simulation.");
  // }
  return PxVec3ToVec3(mPxArticulation->getRootAngularVelocity());
}

void PhysxArticulation::setRootPose(Pose const &pose) {
  // if (getRoot()->isUsingDirectGPUAPI()) {
  //   throw std::runtime_error("setting root pose is not supported in GPU simulation.");
  // }
  mPxArticulation->setRootGlobalPose(PoseToPxTransform(pose));
  syncPose();
}
void PhysxArticulation::setRootLinearVelocity(Vec3 const &v) {
  // if (getRoot()->isUsingDirectGPUAPI()) {
  //   throw std::runtime_error("setting root velocity is not supported in GPU simulation.");
  // }
  mPxArticulation->setRootLinearVelocity(Vec3ToPxVec3(v));
}
void PhysxArticulation::setRootAngularVelocity(Vec3 const &v) {
  // if (getRoot()->isUsingDirectGPUAPI()) {
  //   throw std::runtime_error("setting root angular is not supported in GPU simulation.");
  // }
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

void PhysxArticulation::createFixedTendon(
    std::vector<std::shared_ptr<PhysxArticulationLinkComponent>> const &chain,
    std::vector<float> const &coefficients, std::vector<float> const &recipCoefficients,
    float restLength, float offset, float stiffness, float damping, float low, float high,
    float limitStiffness) {
  for (auto &link : chain) {
    if (link->getArticulation().get() != this) {
      throw std::runtime_error(
          "failed to create tendon: the link does not belong to this articulation.");
    }
  }
  if (chain.size() == 0) {
    throw std::runtime_error("failed to create tendon: no links provided");
  }
  if (chain.size() != coefficients.size() || chain.size() != recipCoefficients.size()) {
    throw std::runtime_error(
        "failed to create tendon: link count does not match one coefficient count");
  }

  std::map<PxArticulationLink *, PxArticulationTendonJoint *> l2t;

  PxScene *scene{};
  if ((scene = mPxArticulation->getScene())) {
    mPxArticulation->getScene()->removeArticulation(*mPxArticulation);
  }
  auto tendon = mPxArticulation->createFixedTendon();

  tendon->setOffset(offset);
  tendon->setRestLength(restLength);
  tendon->setStiffness(stiffness);
  tendon->setDamping(damping);
  tendon->setLimitParameters({low, high});
  tendon->setLimitStiffness(limitStiffness);

  for (uint32_t i = 0; i < chain.size(); ++i) {
    PxArticulationAxis::Enum axis;
    switch (chain.at(i)->getJoint()->getType()) {
    case PxArticulationJointType::ePRISMATIC:
      axis = PxArticulationAxis::eX;
      break;
    case PxArticulationJointType::eREVOLUTE:
    case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
      axis = PxArticulationAxis::eTWIST;
      break;
    default:
      if (i != 0) {
        tendon->release();
        if (scene) {
          scene->addArticulation(*mPxArticulation);
        }
        throw std::runtime_error(
            "failed to create tendon: all joints but the root must be prismatic or revolute");
      }
      axis = PxArticulationAxis::eX; // axis does not matter for root
    }

    auto link = chain.at(i)->getPxActor();
    if (i == 0) {
      l2t[link] = tendon->createTendonJoint(nullptr, axis, coefficients.at(i),
                                            recipCoefficients.at(i), link);
    } else {
      auto parent = link->getInboundJoint() ? &link->getInboundJoint()->getParentArticulationLink()
                                            : nullptr;
      if (!l2t.contains(parent)) {
        tendon->release();
        if (scene) {
          scene->addArticulation(*mPxArticulation);
        }
        throw std::runtime_error("failed to create tendon: links do not form a tree");
      }
      if (l2t.contains(link)) {
        tendon->release();
        if (scene) {
          scene->addArticulation(*mPxArticulation);
        }
        throw std::runtime_error("failed to create tendon: duplicated links");
      }
      l2t[link] = tendon->createTendonJoint(l2t.at(parent), axis, coefficients.at(i),
                                            recipCoefficients.at(i), link);
    }
  }
  if (scene) {
    scene->addArticulation(*mPxArticulation);
  }
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

int PhysxArticulation::getGpuIndex() const {
  return getPxArticulation()->getGpuArticulationIndex();
}

} // namespace physx
} // namespace sapien
