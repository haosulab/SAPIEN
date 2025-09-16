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
#include "sapien/physx/articulation_link_component.h"
#include "../logger.h"
#include "sapien/entity.h"
#include "sapien/math/conversion.h"
#include "sapien/physx/articulation.h"
#include "sapien/physx/joint_component.h"
#include "sapien/physx/physx_system.h"
#include "sapien/scene.h"

#include "sapien/profiler.h"

using namespace physx;

namespace sapien {
namespace physx {

PhysxArticulationJoint::PhysxArticulationJoint(std::weak_ptr<PhysxArticulationLinkComponent> link)
    : mLink(link) {}

void PhysxArticulationJoint::setType(PxArticulationJointType::Enum type) {
  // TODO: make sure the articulation is not added to scene
  // TODO: handle updates to Gear hinge
  mAxes.clear();
  if (auto j = getPxJoint()) {
    j->setJointType(type);
    switch (type) {
    case PxArticulationJointType::ePRISMATIC:
      mAxes = {PxArticulationAxis::eX};
      break;
    case PxArticulationJointType::eREVOLUTE:
    case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
      mAxes = {PxArticulationAxis::eTWIST};
      break;
    case PxArticulationJointType::eSPHERICAL:
      throw std::runtime_error("spherical joint is not supported yet");
      break;
    case PxArticulationJointType::eFIX:
      break;
    default:
      throw std::runtime_error("invalid joint type");
    }
    return;
  }

  if (type != PxArticulationJointType::eFIX && type != PxArticulationJointType::eUNDEFINED) {
    throw std::runtime_error("failed to set joint type: root joint must be either fixed or free");
  }

  // root
  bool fixBase = type == PxArticulationJointType::eFIX;
  mLink.lock()->getArticulation()->getPxArticulation()->setArticulationFlag(
      PxArticulationFlag::eFIX_BASE, fixBase);
}

PxArticulationJointType::Enum PhysxArticulationJoint::getType() const {
  if (auto j = getPxJoint()) {
    return j->getJointType();
  }
  if (mLink.lock()->getArticulation()->getPxArticulation()->getArticulationFlags().isSet(
          PxArticulationFlag::eFIX_BASE)) {
    return PxArticulationJointType::eFIX;
  }
  return PxArticulationJointType::eUNDEFINED;
}

void PhysxArticulationJoint::setAnchorPoseInChild(Pose const &pose) {
  if (auto j = getPxJoint()) {
    j->setChildPose(PoseToPxTransform(pose));
  }
}
Pose PhysxArticulationJoint::getAnchorPoseInChild() const {
  if (auto j = getPxJoint()) {
    return PxTransformToPose(j->getChildPose());
  }
  return Pose();
}
void PhysxArticulationJoint::setAnchorPoseInParent(Pose const &pose) {
  if (auto j = getPxJoint()) {
    j->setParentPose(PoseToPxTransform(pose));
  }
}
Pose PhysxArticulationJoint::getAnchorPoseInParent() const {
  if (auto j = getPxJoint()) {
    return PxTransformToPose(j->getParentPose());
  }
  return Pose();
}

float PhysxArticulationJoint::getFriction() const {
  if (auto j = getPxJoint()) {
    return j->getFrictionCoefficient();
  }
  return 0.f;
}
void PhysxArticulationJoint::setFriction(float friction) {
  if (auto j = getPxJoint()) {
    j->setFrictionCoefficient(friction);
  }
}

float PhysxArticulationJoint::getMaxVelocity() const {
  if (auto j = getPxJoint()) {
    return j->getMaxJointVelocity();
  }
  return 0.f;
}
void PhysxArticulationJoint::setMaxVelocity(float v) {
  if (auto j = getPxJoint()) {
    return j->setMaxJointVelocity(v);
  }
}

Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> PhysxArticulationJoint::getLimit() const {
  Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> result;
  result.resize(mAxes.size(), 2);
  if (auto j = getPxJoint()) {
    for (uint32_t i = 0; i < mAxes.size(); ++i) {
      if (j->getMotion(mAxes[i]) == PxArticulationMotion::eFREE) {
        result.row(i) << -std::numeric_limits<float>::infinity(),
            std::numeric_limits<float>::infinity();
        continue;
      }
      auto limit = j->getLimitParams(mAxes[i]);
      result.row(i) << limit.low, limit.high;
    }
  }
  return result;
}

Eigen::VectorXf PhysxArticulationJoint::getArmature() const {
  Eigen::VectorXf result;
  result.resize(mAxes.size());
  auto j = getPxJoint();
  for (uint32_t i = 0; i < mAxes.size(); ++i) {
    result(i) = j->getArmature(mAxes[i]);
  }
  return result;
}

void PhysxArticulationJoint::setArmature(Eigen::VectorXf const &armature) {
  if (mAxes.size() != static_cast<size_t>(armature.size())) {
    throw std::runtime_error("armature must match joint dof");
  }

  auto j = getPxJoint();
  if (!j) {
    return;
  }
  for (uint32_t i = 0; i < mAxes.size(); ++i) {
    j->setArmature(mAxes[i], armature(i));
  }
}

void PhysxArticulationJoint::setLimit(
    Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> const &limit) {
  if (mAxes.size() != static_cast<size_t>(limit.rows())) {
    throw std::runtime_error("limit must match joint dof");
  }

  auto j = getPxJoint();

  // root
  if (!j) {
    return;
  }

  // not root
  for (uint32_t i = 0; i < mAxes.size(); ++i) {
    float low = limit(i, 0);
    float high = limit(i, 1);
    if (std::isinf(low) && low < 0 && std::isinf(high) && high > 0) {
      if (j->getMotion(mAxes[i]) != PxArticulationMotion::eFREE) {
        j->setMotion(mAxes[i], PxArticulationMotion::eFREE);
      }
      continue;
    }
    if (j->getMotion(mAxes[i]) != PxArticulationMotion::eLIMITED) {
      j->setMotion(mAxes[i], PxArticulationMotion::eLIMITED);
    }
    j->setLimitParams(mAxes[i], {low, high});
  }
}

void PhysxArticulationJoint::setDriveProperties(float stiffness, float damping, float maxForce,
                                                PxArticulationDriveType::Enum type) {
  auto j = getPxJoint();
  for (auto axis : mAxes) {
    j->setDriveParams(axis, {stiffness, damping, maxForce, type});
  }
}

float PhysxArticulationJoint::getDriveStiffness() const {
  if (mAxes.size() == 0) {
    throw std::runtime_error("joint dof cannot be zero");
  }
  return getPxJoint()->getDriveParams(mAxes[0]).stiffness;
}
float PhysxArticulationJoint::getDriveDamping() const {
  if (mAxes.size() == 0) {
    throw std::runtime_error("joint dof cannot be zero");
  }
  return getPxJoint()->getDriveParams(mAxes[0]).damping;
}
float PhysxArticulationJoint::getDriveForceLimit() const {
  if (mAxes.size() == 0) {
    throw std::runtime_error("joint dof cannot be zero");
  }
  return getPxJoint()->getDriveParams(mAxes[0]).maxForce;
}
PxArticulationDriveType::Enum PhysxArticulationJoint::getDriveType() const {
  if (mAxes.size() == 0) {
    throw std::runtime_error("joint dof cannot be zero");
  }
  return getPxJoint()->getDriveParams(mAxes[0]).driveType;
}

Eigen::VectorXf PhysxArticulationJoint::getDriveTargetPosition() const {
  // if (mLink.lock()->isUsingDirectGPUAPI()) {
  //   throw std::runtime_error("failed to access drive: not supported on GPU mode");
  // }
  Eigen::VectorXf result;
  result.resize(mAxes.size());
  auto j = getPxJoint();
  for (uint32_t i = 0; i < mAxes.size(); ++i) {
    result(i) = j->getDriveTarget(mAxes[i]);
  }
  return result;
}

Eigen::VectorXf PhysxArticulationJoint::getDriveTargetVelocity() const {
  // if (mLink.lock()->isUsingDirectGPUAPI()) {
  //   throw std::runtime_error("failed to access drive: not supported on GPU mode");
  // }
  Eigen::VectorXf result;
  result.resize(mAxes.size());
  auto j = getPxJoint();
  for (uint32_t i = 0; i < mAxes.size(); ++i) {
    result(i) = j->getDriveVelocity(mAxes[i]);
  }
  return result;
}

void PhysxArticulationJoint::setDriveTargetPosition(Eigen::VectorXf const &position) {
  // if (mLink.lock()->isUsingDirectGPUAPI()) {
  //   throw std::runtime_error("failed to access drive: not supported on GPU mode");
  // }
  if (static_cast<size_t>(position.size()) != mAxes.size()) {
    throw std::runtime_error("target size does not match joint dof");
  }
  auto j = getPxJoint();
  for (uint32_t i = 0; i < mAxes.size(); ++i) {
    j->setDriveTarget(mAxes[i], position(i));
  }
}

void PhysxArticulationJoint::setDriveTargetVelocity(Eigen::VectorXf const &velocity) {
  // if (mLink.lock()->isUsingDirectGPUAPI()) {
  //   throw std::runtime_error("failed to access drive: not supported on GPU mode");
  // }
  if (static_cast<size_t>(velocity.size()) != mAxes.size()) {
    throw std::runtime_error("target size does not match joint dof");
  }
  auto j = getPxJoint();
  for (uint32_t i = 0; i < mAxes.size(); ++i) {
    j->setDriveVelocity(mAxes[i], velocity(i));
  }
}

void PhysxArticulationJoint::setDriveTargetPosition(float position) {
  // if (mLink.lock()->isUsingDirectGPUAPI()) {
  //   throw std::runtime_error("failed to access drive: not supported on GPU mode");
  // }
  Eigen::VectorXf v(1);
  v << position;
  setDriveTargetPosition(v);
}

void PhysxArticulationJoint::setDriveTargetVelocity(float velocity) {
  // if (mLink.lock()->isUsingDirectGPUAPI()) {
  //   throw std::runtime_error("failed to access drive: not supported on GPU mode");
  // }
  Eigen::VectorXf v(1);
  v << velocity;
  setDriveTargetVelocity(v);
}

uint32_t PhysxArticulationJoint::getDof() const { return mAxes.size(); }

std::shared_ptr<PhysxArticulationLinkComponent> PhysxArticulationJoint::getParentLink() const {
  auto l = mLink.lock();
  if (!l) {
    throw std::runtime_error("the articulation of the joint has been destroyed");
  }
  return l->getParent();
}

std::shared_ptr<PhysxArticulationLinkComponent> PhysxArticulationJoint::getChildLink() const {
  auto l = mLink.lock();
  if (!l) {
    throw std::runtime_error("the articulation of the joint has been destroyed");
  }
  return l;
}

Pose PhysxArticulationJoint::getGlobalPose() const {
  return getChildLink()->getPose() * getAnchorPoseInChild();
}

PxArticulationJointReducedCoordinate *PhysxArticulationJoint::getPxJoint() const {
  auto l = mLink.lock();
  if (!l) {
    throw std::runtime_error("the articulation of the joint has been destroyed");
  }
  return l->getPxActor()->getInboundJoint();
}

std::shared_ptr<PhysxArticulationLinkComponent>
PhysxArticulationLinkComponent::Create(std::shared_ptr<PhysxArticulationLinkComponent> parent) {
  auto link = std::make_shared<PhysxArticulationLinkComponent>(parent);

  if (!parent) {
    link->mArticulation = std::make_shared<PhysxArticulation>();
  } else {
    link->mArticulation = parent->getArticulation();
  }

  link->mArticulation->internalEnsureRemovedFromScene();
  link->mArticulation->addLink(*link, parent.get());
  // the current articulation is definitely not in a scene

  link->mJoint = std::make_shared<PhysxArticulationJoint>(link);

  // update references
  if (parent) {
    parent->mChildren.push_back(link);
  }
  link->mParent = parent;
  if (parent) {
    link->getJoint()->setType(PxArticulationJointType::eFIX);
  }
  return link;
}

PhysxArticulationLinkComponent::PhysxArticulationLinkComponent(
    std::shared_ptr<PhysxArticulationLinkComponent> parent)
    : mParent(parent) {}

bool PhysxArticulationLinkComponent::isRoot() const { return mParent == nullptr; }

void PhysxArticulationLinkComponent::onAddToScene(Scene &scene) {
  mArticulation->internalNotifyAddToScene(this, scene);
  auto system = scene.getPhysxSystem();

  system->registerComponent(
      std::static_pointer_cast<PhysxArticulationLinkComponent>(shared_from_this()));
}
void PhysxArticulationLinkComponent::onRemoveFromScene(Scene &scene) {
  mArticulation->internalNotifyRemoveFromScene(this, scene);
  auto system = scene.getPhysxSystem();
  system->unregisterComponent(
      std::static_pointer_cast<PhysxArticulationLinkComponent>(shared_from_this()));
}
void PhysxArticulationLinkComponent::onSetPose(Pose const &pose) {
  if (!isUsingDirectGPUAPI()) {
    if (isRoot()) {
      mArticulation->setRootPose(pose);
    }
  } else {
    // TODO: allow disabling this warning
    logger::warn("setting pose has no effect for physx GPU simulation.");
  }
}
void PhysxArticulationLinkComponent::syncPoseToEntity() {
  // TODO: how slow is it? get on demand?
  getEntity()->internalSyncPose(PxTransformToPose(getPxActor()->getGlobalPose()));
}

void PhysxArticulationLinkComponent::setParent(
    std::shared_ptr<PhysxArticulationLinkComponent> parent) {
  auto currentParent = getParent();
  if (parent == currentParent) {
    return;
  }

  // update references
  if (currentParent) {
    std::erase_if(currentParent->mChildren, [this](auto c) {
      return c.lock() ==
             std::static_pointer_cast<PhysxArticulationLinkComponent>(shared_from_this());
    });
  }
  mParent = parent;

  auto newArticulation =
      parent ? parent->getArticulation() : std::make_shared<PhysxArticulation>();

  // handle children
  auto links = mArticulation->findDescendants(
      std::static_pointer_cast<PhysxArticulationLinkComponent>(shared_from_this()));

  // cache property of links here
  struct LinkProperties {
    PxActorFlags actorFlag;
    PxRigidBodyFlags rigidFlag;
    float mass{};
    PxVec3 inertia{};
    PxTransform cmassLocalPose{};
    float linearDamping;
    float angularDamping;

    PxTransform linkPose{};
    PxVec3 velocity{};
    PxVec3 angularVelocity{};

    PxArticulationJointType::Enum jointType{};
    float jointFriction{};
    PxTransform jointParentPose{};
    PxTransform jointChildPose{};

    PxArticulationMotion::Enum jointMotion[6]{};
    PxArticulationLimit jointLimit[6]{};
    PxArticulationDrive jointDrive[6]{};
    float jointDriveTarget[6]{};
    float jointDriveVelocity[6]{};
    float jointPosition[6]{};
    float jointVelocity[6]{};
    float jointArmature[6]{};
  };

  std::vector<LinkProperties> linkInfo;
  linkInfo.reserve(links.size());
  for (auto l : links) {
    auto pxlink = l->getPxActor();
    auto pxjoint = pxlink->getInboundJoint();
    LinkProperties property{
        .actorFlag = pxlink->getActorFlags(),
        .rigidFlag = pxlink->getRigidBodyFlags(),
        .mass = pxlink->getMass(),
        .inertia = pxlink->getMassSpaceInertiaTensor(),
        .cmassLocalPose = pxlink->getCMassLocalPose(),
        .linearDamping = pxlink->getLinearDamping(),
        .angularDamping = pxlink->getAngularDamping(),
        .linkPose = pxlink->getGlobalPose(),
        .velocity = pxlink->getLinearVelocity(),
        .angularVelocity = pxlink->getAngularVelocity(),
        .jointType = pxjoint ? pxjoint->getJointType() : PxArticulationJointType::eUNDEFINED,
    };

    if (pxjoint) {
      property.jointFriction = pxjoint->getFrictionCoefficient();
      property.jointParentPose = pxjoint->getParentPose();
      property.jointChildPose = pxjoint->getChildPose();
      std::vector<PxArticulationAxis::Enum> axes = {
          PxArticulationAxis::eTWIST, PxArticulationAxis::eSWING1, PxArticulationAxis::eSWING2,
          PxArticulationAxis::eX,     PxArticulationAxis::eY,      PxArticulationAxis::eZ,
      };
      for (int i = 0; i < 6; ++i) {
        property.jointMotion[i] = pxjoint->getMotion(axes[i]);
        property.jointLimit[i] = pxjoint->getLimitParams(axes[i]);
        property.jointDrive[i] = pxjoint->getDriveParams(axes[i]);
        property.jointDriveTarget[i] = pxjoint->getDriveTarget(axes[i]);
        property.jointDriveVelocity[i] = pxjoint->getDriveVelocity(axes[i]);
        property.jointPosition[i] = pxjoint->getJointPosition(axes[i]);
        property.jointVelocity[i] = pxjoint->getJointVelocity(axes[i]);
        property.jointArmature[i] = pxjoint->getArmature(axes[i]);
      }
    }
    linkInfo.push_back(property);
  }

  mArticulation->internalEnsureRemovedFromScene();
  newArticulation->internalEnsureRemovedFromScene();

  // remove links from old articulation in reverse order
  for (auto it = links.rbegin(); it != links.rend(); ++it) {
    auto l = *it;
    if (l->mParent) {
      std::erase_if(l->mParent->mChildren, [l](auto c) { return c.lock() == l; });
    }
    mArticulation->removeLink(*l);
  }

  std::shared_ptr<PhysxArticulation> currentArticulation = mArticulation;

  // TODO: remove articulation if there are no links
  if (currentParent) {
    // add articulation back to scene if the detached node is not root
    currentArticulation->internalEnsureAddedToScene();
  }

  // add links to the new articulation
  for (uint32_t i = 0; i < links.size(); ++i) {
    newArticulation->addLink(*links[i], links[i]->getParent().get());
    links[i]->mArticulation = newArticulation;
    for (auto s : links[i]->mCollisionShapes) {
      links[i]->mPxLink->attachShape(*s->getPxShape());
    }
  }

  // restore properties of the joints
  for (uint32_t i = 0; i < links.size(); ++i) {
    auto pxlink = links[i]->getPxActor();
    auto pxjoint = pxlink->getInboundJoint();

    auto &info = linkInfo[i];
    pxlink->setActorFlags(info.actorFlag);
    pxlink->setRigidBodyFlags(info.rigidFlag);
    pxlink->setMass(info.mass);
    pxlink->setMassSpaceInertiaTensor(info.inertia);
    pxlink->setCMassLocalPose(info.cmassLocalPose);
    pxlink->setLinearDamping(info.linearDamping);
    pxlink->setAngularDamping(info.angularDamping);

    auto joint = links[i]->getJoint();
    if (pxjoint) {
      joint->setType(info.jointType == PxArticulationJointType::eUNDEFINED
                         ? PxArticulationJointType::eFIX
                         : info.jointType);
      pxjoint->setFrictionCoefficient(info.jointFriction);
      pxjoint->setParentPose(info.jointParentPose);
      pxjoint->setChildPose(info.jointChildPose);
      std::vector<PxArticulationAxis::Enum> axes = {
          PxArticulationAxis::eTWIST, PxArticulationAxis::eSWING1, PxArticulationAxis::eSWING2,
          PxArticulationAxis::eX,     PxArticulationAxis::eY,      PxArticulationAxis::eZ,
      };
      for (int i = 0; i < 6; ++i) {
        pxjoint->setMotion(axes[i], info.jointMotion[i]);
        pxjoint->setLimitParams(axes[i], info.jointLimit[i]);
        pxjoint->setDriveParams(axes[i], info.jointDrive[i]);
        pxjoint->setDriveTarget(axes[i], info.jointDriveTarget[i]);
        pxjoint->setDriveVelocity(axes[i], info.jointDriveVelocity[i]);
        pxjoint->setJointPosition(axes[i], info.jointPosition[i]);
        pxjoint->setJointVelocity(axes[i], info.jointVelocity[i]);
        pxjoint->setArmature(axes[i], info.jointArmature[i]);
      }
    } else {
      bool fixBase = info.jointType == PxArticulationJointType::eFIX;
      newArticulation->getPxArticulation()->setArticulationFlag(PxArticulationFlag::eFIX_BASE,
                                                                fixBase);
      newArticulation->getPxArticulation()->setRootGlobalPose(info.linkPose);
      // TODO: sync pose with sapien
      newArticulation->getPxArticulation()->setRootLinearVelocity(info.velocity);
      newArticulation->getPxArticulation()->setRootAngularVelocity(info.angularVelocity);
    }
  }
  newArticulation->internalEnsureAddedToScene();

  // refresh joints since underlying actor changed for links
  for (auto l : links) {
    for (auto d : l->mJoints) {
      if (auto drive = d.lock()) {
        drive->internalRefresh();
      }
    }
  }
}

int PhysxArticulationLinkComponent::getIndex() const {
  if (!mArticulation->getPxArticulation()->getScene()) {
    throw std::runtime_error(
        "failed to get link index: the articulation first needs to be added to a scene.");
  }
  return getPxActor()->getLinkIndex();
}

bool PhysxArticulationLinkComponent::isSleeping() const {
  return getArticulation()->getPxArticulation()->isSleeping();
}
void PhysxArticulationLinkComponent::wakeUp() {
  return getArticulation()->getPxArticulation()->wakeUp();
}
void PhysxArticulationLinkComponent::putToSleep() {
  return getArticulation()->getPxArticulation()->putToSleep();
}

PhysxArticulationLinkComponent::~PhysxArticulationLinkComponent() {
  // children own parent
  assert(mChildren.size() == 0);

  // this is now an expired weak_ptr in parent
  if (mParent) {
    std::erase_if(mParent->mChildren, [](auto c) { return c.expired(); });
  }
}

std::vector<std::shared_ptr<PhysxArticulationLinkComponent>>
PhysxArticulationLinkComponent::cloneArticulation(
    std::shared_ptr<PhysxArticulationLinkComponent> root) {

  SAPIEN_PROFILE_FUNCTION

  if (!root->isRoot()) {
    throw std::runtime_error("clone articulation takes a root link");
  }

  std::map<std::shared_ptr<PhysxArticulationLinkComponent>,
           std::shared_ptr<PhysxArticulationLinkComponent>>
      old2new;

  auto links = root->getArticulation()->getLinksAdditionOrder();
  std::vector<std::shared_ptr<PhysxArticulationLinkComponent>> newLinks;

  old2new[root] = PhysxArticulationLinkComponent::Create();
  for (auto link : links) {
    auto parent = link->getParent() ? old2new.at(link->getParent()) : nullptr;

    auto newLink = PhysxArticulationLinkComponent::Create(parent);

    newLinks.push_back(newLink);
    old2new[link] = newLink;

    // TODO: move to rigid body class

    // update mass
    newLink->setAutoComputeMass(false);

    for (auto s : link->getCollisionShapes()) {
      newLink->attachCollision(s->clone());
    }

    newLink->setMass(link->getMass());
    newLink->setInertia(link->getInertia());
    newLink->setCMassLocalPose(link->getCMassLocalPose());
    newLink->mAutoComputeMass = link->getAutoComputeMass();

    // other properties
    newLink->setName(link->getName());
    newLink->setAngularDamping(link->getAngularDamping());
    newLink->setLinearDamping(link->getLinearDamping());
    newLink->setDisableGravity(link->getDisableGravity());
    newLink->setMaxContactImpulse(link->getMaxContactImpulse());
    newLink->setMaxDepenetrationVelocity(link->getMaxDepenetrationVelocity());

    auto joint = link->getJoint();
    auto newJoint = newLink->getJoint();
    newJoint->setName(joint->getName());
    newJoint->setType(joint->getType());
    newJoint->setAnchorPoseInChild(joint->getAnchorPoseInChild());
    newJoint->setAnchorPoseInParent(joint->getAnchorPoseInParent());
    newJoint->setLimit(joint->getLimit());
    newJoint->setArmature(joint->getArmature());
    newJoint->setFriction(joint->getFriction());
    newJoint->setDriveTargetPosition(joint->getDriveTargetPosition());
    newJoint->setDriveTargetVelocity(joint->getDriveTargetVelocity());
    if (joint->getDof() != 0) {
      newJoint->setDriveProperties(newJoint->getDriveStiffness(), newJoint->getDriveDamping(),
                                   newJoint->getDriveForceLimit(), newJoint->getDriveType());
    }
  }

  auto art = root->getArticulation();
  auto newArt = newLinks.at(0)->getArticulation();
  newArt->setName(art->getName());
  newArt->setRootPose(art->getRootPose());
  newArt->setRootLinearVelocity(art->getRootLinearVelocity());
  newArt->setRootAngularVelocity(art->getRootAngularVelocity());

  ProfilerBlockEnd();

  return newLinks;
}

} // namespace physx
} // namespace sapien
