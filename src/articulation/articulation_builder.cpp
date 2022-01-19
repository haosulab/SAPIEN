#include "articulation_builder.h"
#include "sapien_articulation.h"
#include "sapien_joint.h"
#include "sapien_kinematic_articulation.h"
#include "sapien_kinematic_joint.h"
#include "sapien_link.h"
#include "sapien_scene.h"
#include "simulation.h"
#include <spdlog/spdlog.h>

namespace sapien {

LinkBuilder::LinkBuilder(ArticulationBuilder *articulationBuilder, int index, int parentIndex)
    : ActorBuilder(articulationBuilder->getScene()), mArticulationBuilder(articulationBuilder),
      mIndex(index), mParent(parentIndex) {}

void LinkBuilder::setJointName(const std::string &jointName) { mJointRecord.name = jointName; }
std::string LinkBuilder::getJointName() { return mJointRecord.name; }

void LinkBuilder::setJointProperties(PxArticulationJointType::Enum jointType,
                                     std::vector<std::array<PxReal, 2>> const &limits,
                                     PxTransform const &parentPose, PxTransform const &childPose,
                                     PxReal friction, PxReal damping) {
  mJointRecord.jointType = jointType;
  mJointRecord.limits = limits;
  mJointRecord.parentPose = parentPose;
  mJointRecord.childPose = childPose;
  mJointRecord.friction = friction;
  mJointRecord.damping = damping;
}

std::string LinkBuilder::summary() const {
  std::stringstream ss;
  ss << "Link " << getIndex() << ". \"" << mName << "\". "
     << "Parent " << mParent << std::endl;
  ss << "  \"" + mJointRecord.name + "\" ";

  if (mParent < 0) {
    ss << "Virtual.";
  } else {
    switch (mJointRecord.jointType) {
    case PxArticulationJointType::eFIX:
      ss << "Fixed.";
      if (!checkJointProperties()) {
        ss << " Not valid.";
      }
      break;
    case PxArticulationJointType::eREVOLUTE:
      ss << "Revolute.";
      if (!checkJointProperties()) {
        ss << " Not valid.";
      } else {
        if (mJointRecord.limits[0][0] == -std::numeric_limits<float>::infinity()) {
          ss << " Free.";
        } else {
          ss << " [" << mJointRecord.limits[0][0] << ",  " << mJointRecord.limits[0][1] << "]";
        }
      }
      break;
    case PxArticulationJointType::ePRISMATIC:
      ss << "ePrismatic.";
      if (!checkJointProperties()) {
        ss << " Not valid.";
      } else {
        if (mJointRecord.limits[0][0] == -std::numeric_limits<float>::infinity()) {
          ss << " Free.";
        } else {
          ss << " " << mJointRecord.limits[0][0] << " - " << mJointRecord.limits[0][1];
        }
      }
      break;
    case PxArticulationJointType::eUNDEFINED:
      ss << "Virtual.";
      if (!checkJointProperties()) {
        ss << " Not valid.";
      }
      break;
    case PxArticulationJointType::eSPHERICAL:
      ss << "Spherical. Unsupported";
      break;
    }
  }
  return ss.str();
}

bool LinkBuilder::checkJointProperties() const {
  if (!mJointRecord.parentPose.isSane()) {
    spdlog::get("SAPIEN")->error("Invalid parent pose for joint {}. \"{}\"", mIndex,
                                 mJointRecord.name);
    return false;
  }
  if (!mJointRecord.childPose.isSane()) {
    spdlog::get("SAPIEN")->error("Invalid child pose for joint {}. \"{}\"", mIndex,
                                 mJointRecord.name);
    return false;
  }

  switch (mJointRecord.jointType) {
  case PxArticulationJointType::eFIX: {
    if (mJointRecord.limits.size() != 0) {
      spdlog::get("SAPIEN")->error("Fixed joint should have 0 limits for joint {}. \"{}\"", mIndex,
                                   mJointRecord.name);
      return false;
    }
    return true;
  }
  case PxArticulationJointType::eREVOLUTE: {
    if (mJointRecord.limits.size() != 1) {
      spdlog::get("SAPIEN")->error("Revolute joint should have 1 limits for joint {}. \"{}\"",
                                   mIndex, mJointRecord.name);
      return false;
    }
    return true;
  }
  case PxArticulationJointType::ePRISMATIC: {
    if (mJointRecord.limits.size() != 1) {
      spdlog::get("SAPIEN")->error("Prismatic joint should have 1 limits for joint {}. \"{}\"",
                                   mIndex, mJointRecord.name);
      return false;
    }
    return true;
  }
  default:
    spdlog::get("SAPIEN")->error("Unsupported joint type for joint {}. \"{}\"", mIndex,
                                 mJointRecord.name);
    return false;
  }
}

bool LinkBuilder::build(SArticulation &articulation) const {
  auto pxArticulation = articulation.mPxArticulation;
  auto &links = articulation.mLinks;
  auto &joints = articulation.mJoints;

  // create link
  physx_id_t linkId = mArticulationBuilder->getScene()->mActorIdGenerator.next();
  PxArticulationLink *pxLink = pxArticulation->createLink(
      mParent >= 0 ? links[mParent]->getPxActor() : nullptr, {{0, 0, 0}, PxIdentity});

  std::vector<std::unique_ptr<SCollisionShape>> shapes;
  std::vector<PxReal> densities;
  buildShapes(shapes, densities);

  std::vector<physx_id_t> renderIds;
  std::vector<Renderer::IPxrRigidbody *> renderBodies;
  buildVisuals(renderBodies, renderIds);
  for (auto body : renderBodies) {
    body->setSegmentationId(linkId);
  }

  std::vector<Renderer::IPxrRigidbody *> collisionBodies;
  buildCollisionVisuals(collisionBodies, shapes);
  for (auto body : collisionBodies) {
    body->setSegmentationId(linkId);
  }

  PxFilterData data;
  data.word0 = mCollisionGroup.w0;
  data.word1 = mCollisionGroup.w1;
  data.word2 = mCollisionGroup.w2;
  data.word3 = 0;

  // wrap link
  links[mIndex] = std::unique_ptr<SLink>(new SLink(pxLink, &articulation, linkId,
                                                   mArticulationBuilder->getScene(), renderBodies,
                                                   collisionBodies));

  for (size_t i = 0; i < shapes.size(); ++i) {
    shapes[i]->setCollisionGroups(mCollisionGroup.w0, mCollisionGroup.w1, mCollisionGroup.w2,
                                  mCollisionGroup.w3);
    links[mIndex]->attachShape(std::move(shapes[i]));
  }

  if (shapes.size() && mUseDensity) {
    bool zero = true;
    for (float density : densities) {
      if (density > 1e-8) {
        zero = false;
        break;
      }
    }
    if (zero) {
      spdlog::get("SAPIEN")->warn(
          "All shapes have 0 density. This will result in unexpected mass and inertia.");
    }
    PxRigidBodyExt::updateMassAndInertia(*pxLink, densities.data(), shapes.size());
  } else {
    if (mMass < 1e-6 || mInertia.x < 1e-8 || mInertia.y < 1e-8 || mInertia.z < 1e-8) {
      spdlog::get("SAPIEN")->info(
          "Mass or inertia contains very small number, this is not allowed. "
          "Mass will be set to 1e-6 and inertia will be set to 1e-8 for stability. Link: {0}",
          mName);
      pxLink->setMass(1e-6);
      pxLink->setMassSpaceInertiaTensor({1e-8, 1e-8, 1e-8});
    } else {
      pxLink->setMass(mMass);
      pxLink->setCMassLocalPose(mCMassPose);
      pxLink->setMassSpaceInertiaTensor(mInertia);
    }
  }

  links[mIndex]->setName(mName);

  links[mIndex]->mCol1 = mCollisionGroup.w0;
  links[mIndex]->mCol2 = mCollisionGroup.w1;
  links[mIndex]->mCol3 = mCollisionGroup.w2;
  links[mIndex]->mIndex = mIndex;

  pxLink->userData = links[mIndex].get();

  // create and wrap joint
  auto joint = static_cast<PxArticulationJointReducedCoordinate *>(pxLink->getInboundJoint());
  std::unique_ptr<SJoint> j;
  if (joint) {
    joint->setJointType(mJointRecord.jointType);
    joint->setParentPose(mJointRecord.parentPose);
    joint->setChildPose(mJointRecord.childPose);
    j = std::unique_ptr<SJoint>(
        new SJoint(&articulation, links[mParent].get(), links[mIndex].get(), joint));
    j->setLimits(mJointRecord.limits);
    j->setFriction(mJointRecord.friction);
    j->setDriveProperty(0, mJointRecord.damping);
  } else {
    j = std::unique_ptr<SJoint>(new SJoint(&articulation, nullptr, links[mIndex].get(), nullptr));
  }
  j->setName(mJointRecord.name);
  joints[mIndex] = std::move(j);

  return true;
}

bool LinkBuilder::buildKinematic(SKArticulation &articulation) const {
  auto &links = articulation.mLinks;
  auto &joints = articulation.mJoints;

  physx_id_t linkId = mScene->mActorIdGenerator.next();

  std::vector<std::unique_ptr<SCollisionShape>> shapes;
  std::vector<PxReal> densities;
  buildShapes(shapes, densities);

  std::vector<physx_id_t> renderIds;
  std::vector<Renderer::IPxrRigidbody *> renderBodies;
  buildVisuals(renderBodies, renderIds);
  for (auto body : renderBodies) {
    body->setSegmentationId(linkId);
  }

  std::vector<Renderer::IPxrRigidbody *> collisionBodies;
  buildCollisionVisuals(collisionBodies, shapes);
  for (auto body : collisionBodies) {
    body->setSegmentationId(linkId);
  }

  PxFilterData data;
  data.word0 = mCollisionGroup.w0;
  data.word1 = mCollisionGroup.w1;
  data.word2 = mCollisionGroup.w2;
  data.word3 = 0;

  PxRigidDynamic *actor =
      mScene->getSimulation()->mPhysicsSDK->createRigidDynamic(PxTransform(PxIdentity));
  actor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
  links[mIndex] = std::unique_ptr<SKLink>(new SKLink(actor, &articulation, linkId,
                                                     mArticulationBuilder->getScene(),
                                                     renderBodies, collisionBodies));
  for (size_t i = 0; i < shapes.size(); ++i) {
    shapes[i]->setCollisionGroups(mCollisionGroup.w0, mCollisionGroup.w1, mCollisionGroup.w2,
                                  mCollisionGroup.w3);
    links[mIndex]->attachShape(std::move(shapes[i]));
  }

  if (shapes.size() && mUseDensity) {
    PxRigidBodyExt::updateMassAndInertia(*actor, densities.data(), shapes.size());
  } else {
    if (mMass < 1e-8 || mInertia.x < 1e-8 || mInertia.y < 1e-8 || mInertia.z < 1e-8) {
      actor->setMass(1e-6);
      actor->setMassSpaceInertiaTensor({1e-6, 1e-6, 1e-6});
    } else {
      actor->setMass(mMass);
      actor->setCMassLocalPose(mCMassPose);
      actor->setMassSpaceInertiaTensor(mInertia);
    }
  }

  links[mIndex]->setName(mName);

  links[mIndex]->mCol1 = mCollisionGroup.w0;
  links[mIndex]->mCol2 = mCollisionGroup.w1;
  links[mIndex]->mCol3 = mCollisionGroup.w2;
  links[mIndex]->mIndex = mIndex;

  actor->userData = links[mIndex].get();

  std::unique_ptr<SKJoint> j;
  if (mParent >= 0) {
    switch (mJointRecord.jointType) {
    case PxArticulationJointType::eFIX:
      j = std::unique_ptr<SKJoint>(
          new SKJointFixed(&articulation, links[mParent].get(), links[mIndex].get()));
      break;
    case PxArticulationJointType::eREVOLUTE:
      j = std::unique_ptr<SKJoint>(
          new SKJointRevolute(&articulation, links[mParent].get(), links[mIndex].get()));
      break;
    case PxArticulationJointType::ePRISMATIC:
      j = std::unique_ptr<SKJoint>(
          new SKJointPrismatic(&articulation, links[mParent].get(), links[mIndex].get()));
      break;
    default:
      spdlog::get("SAPIEN")->error("Unsupported kinematic joint type");
    }
    j->setLimits(mJointRecord.limits);
  } else {
    j = std::unique_ptr<SKJoint>(new SKJointFixed(&articulation, nullptr, links[mIndex].get()));
  }
  j->setParentPose(mJointRecord.parentPose);
  j->setChildPose(mJointRecord.childPose);
  j->setName(mJointRecord.name);
  joints[mIndex] = std::move(j);

  return true;
}

ArticulationBuilder::ArticulationBuilder(SScene *scene) : mScene(scene) {}

std::shared_ptr<LinkBuilder>
ArticulationBuilder::createLinkBuilder(std::shared_ptr<LinkBuilder> parent) {
  return createLinkBuilder(parent ? parent->mIndex : -1);
}

std::shared_ptr<LinkBuilder> ArticulationBuilder::createLinkBuilder(int parentIdx) {
  mLinkBuilders.push_back(std::make_shared<LinkBuilder>(this, mLinkBuilders.size(), parentIdx));
  return mLinkBuilders.back();
}

std::string ArticulationBuilder::summary() const {
  std::stringstream ss;
  ss << "======= Link Summary =======" << std::endl;
  for (auto &b : mLinkBuilders) {
    ss << b->summary() << std::endl;
  }
  ss << "======= Joint summary =======" << std::endl;
  return ss.str();
}

bool ArticulationBuilder::prebuild(std::vector<int> &tosort) const {
  // find tree root
  int root = -1;
  for (auto &b : mLinkBuilders) {
    if (b->mParent < 0) {
      if (root >= 0) {
        spdlog::get("SAPIEN")->error("Failed to build articulation: multiple roots");
        return false;
      }
      root = b->mIndex;
    }
  }

  // root -> children
  std::vector<std::vector<int>> childMap(mLinkBuilders.size());
  for (auto &b : mLinkBuilders) {
    if (b->mParent >= 0) {
      childMap[b->mParent].push_back(b->mIndex);
    }
  }

  // tree traversal
  std::vector<int> visited(mLinkBuilders.size());
  std::vector<int> stack = {root};
  while (!stack.empty()) {
    int elem = stack.back();
    stack.pop_back();
    tosort.push_back(elem);
    if (visited[elem]) {
      spdlog::get("SAPIEN")->error("Failed to build articulation: kinematic loop");
      return false;
    }
    visited[elem] = 1;
    for (int child : childMap[elem]) {
      stack.push_back(child);
    }
  }

  for (auto &builder : mLinkBuilders) {
    if (!builder->checkJointProperties()) {
      spdlog::get("SAPIEN")->error("Failed to build articulation: invalid joint");
      return false;
    }
  }

  return true;
}

SArticulation *ArticulationBuilder::build(bool fixBase) const {
  std::vector<int> sorted;
  if (!prebuild(sorted)) {
    return nullptr;
  }

  auto sArticulation = std::unique_ptr<SArticulation>(new SArticulation(mScene));
  sArticulation->mPxArticulation =
      mScene->getSimulation()->mPhysicsSDK->createArticulationReducedCoordinate();
  sArticulation->mPxArticulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, fixBase);

  sArticulation->mLinks.resize(mLinkBuilders.size());
  sArticulation->mJoints.resize(mLinkBuilders.size());

  // sorted now is topologically sorted
  for (int i : sorted) {
    if (!mLinkBuilders[i]->build(*sArticulation)) {
      sArticulation.release();
      return nullptr;
    }
  }

  auto result = sArticulation.get();
  mScene->addArticulation(std::move(sArticulation));

  {
    uint32_t totalLinkCount = result->mLinks.size();
    std::vector<uint32_t> dofStarts(totalLinkCount); // link dof starts, internal order
    dofStarts[0] = 0;
    for (auto &link : result->mLinks) {
      auto pxLink = link->getPxActor();
      auto idx = pxLink->getLinkIndex();
      if (idx) {
        dofStarts[idx] = pxLink->getInboundJointDof();
      }
    }

    uint32_t count = 0;
    for (uint32_t i = 1; i < totalLinkCount; ++i) {
      uint32_t dofs = dofStarts[i];
      dofStarts[i] = count;
      count += dofs;
    }

    std::vector<uint32_t> E2I;
    count = 0;
    for (uint32_t i = 0; i < totalLinkCount; ++i) {
      uint32_t dof = result->getBaseJoints()[i]->getDof();
      uint32_t start = dofStarts[result->mLinks[i]->getPxActor()->getLinkIndex()];
      for (uint32_t d = 0; d < dof; ++d) {
        E2I.push_back(start + d);
      }
    }

    std::vector<uint32_t> I2E(E2I.size());
    for (uint32_t i = 0; i < E2I.size(); ++i) {
      I2E[E2I[i]] = i;
    }
    result->mIndexE2I = E2I;
    result->mIndexI2E = I2E;
    result->mColumnPermutationI2E = sapien::SArticulation::buildColumnPermutation(I2E);
    result->mRowPermutationI2E = result->buildRowPermutation();
  }

  for (auto &j : result->mJoints) {
    if (!j->getParentLink()) {
      result->mRootLink = static_cast<SLink *>(j->getChildLink());
    }
  }

  result->mCache = result->mPxArticulation->createCache();
  result->mPxArticulation->zeroCache(*result->mCache);

  result->mPxArticulation->setSleepThreshold(mScene->mDefaultSleepThreshold);
  result->mPxArticulation->setSolverIterationCounts(mScene->mDefaultSolverIterations,
                                                    mScene->mDefaultSolverVelocityIterations);

  result->mPxArticulation->setArticulationFlag(PxArticulationFlag::eDRIVE_LIMITS_ARE_FORCES, true);

  // make sure qvel is 0
  std::vector<PxReal> qvel(result->dof(), 0);
  result->setQvel(qvel);

  result->mBuilder = shared_from_this();

  return result;
}

SKArticulation *ArticulationBuilder::buildKinematic() const {
  std::vector<int> sorted;
  if (!prebuild(sorted)) {
    return nullptr;
  }

  auto articulation = std::unique_ptr<SKArticulation>(new SKArticulation(mScene));
  articulation->mLinks.resize(mLinkBuilders.size());
  articulation->mJoints.resize(mLinkBuilders.size());

  for (int i : sorted) {
    if (!mLinkBuilders[i]->buildKinematic(*articulation)) {
      // release resources for links that are already built
      for (auto &link : articulation->mLinks) {
        if (link) {
          for (auto body : link->getRenderBodies()) {
            body->destroy();
          }
          for (auto body : link->getCollisionBodies()) {
            body->destroy();
          }
          link->getPxActor()->release();
        }
      }
      return nullptr;
    }
  }

  auto result = articulation.get();
  mScene->addKinematicArticulation(std::move(articulation));

  result->mDof = 0;
  for (auto &j : result->mJoints) {
    result->mDof += j->getDof();
  }
  result->mSortedIndices = sorted;

  result->mRootLink = static_cast<SKLink *>(result->mJoints[sorted[0]]->getChildLink());

  result->mBuilder = shared_from_this();

  return result;
}

std::vector<LinkBuilder *> ArticulationBuilder::getLinkBuilders() {
  std::vector<LinkBuilder *> builders;
  for (auto &b : mLinkBuilders) {
    builders.push_back(b.get());
  }
  return builders;
}

} // namespace sapien
