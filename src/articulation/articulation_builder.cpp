#include "articulation_builder.h"
#include "sapien_articulation.h"
#include "sapien_joint.h"
#include "sapien_link.h"
#include "sapien_scene.h"
#include "simulation.h"
#include <spdlog/spdlog.h>

namespace sapien {

LinkBuilder::LinkBuilder(ArticulationBuilder *articulationBuilder, int index, int parentIndex)
    : ActorBuilder(articulationBuilder->getScene()), mArticulationBuilder(articulationBuilder),
      mIndex(index), mParent(parentIndex) {}

void LinkBuilder::setJointName(const std::string &jointName) { mJointRecord.name = jointName; }

void LinkBuilder::setJointProperties(PxArticulationJointType::Enum jointType,
                                     const std::vector<std::array<float, 2>> &limits,
                                     const PxTransform &parentPose, const PxTransform &childPose,
                                     const std::string &jointName) {
  mJointRecord.jointType = jointType;
  mJointRecord.limits = limits;
  mJointRecord.parentPose = parentPose;
  mJointRecord.childPose = childPose;
  mJointRecord.name = jointName;
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
    spdlog::error("Invalid parent pose for joint {}. \"{}\"", mIndex, mJointRecord.name);
    return false;
  }
  if (!mJointRecord.childPose.isSane()) {
    spdlog::error("Invalid child pose for joint {}. \"{}\"", mIndex, mJointRecord.name);
    return false;
  }

  switch (mJointRecord.jointType) {
  case PxArticulationJointType::eFIX: {
    if (mJointRecord.limits.size() != 0) {
      spdlog::error("Fixed joint should have 0 limits for joint {}. \"{}\"", mIndex,
                    mJointRecord.name);
      return false;
    }
    return true;
  }
  case PxArticulationJointType::eREVOLUTE: {
    if (mJointRecord.limits.size() != 1) {
      spdlog::error("Revolute joint should have 1 limits for joint {}. \"{}\"", mIndex,
                    mJointRecord.name);
      return false;
    }
    return true;
  }
  case PxArticulationJointType::ePRISMATIC: {
    if (mJointRecord.limits.size() != 1) {
      spdlog::error("Prismatic joint should have 1 limits for joint {}. \"{}\"", mIndex,
                    mJointRecord.name);
      return false;
    }
    return true;
  }
  default:
    spdlog::error("Unsupported joint type for joint {}. \"{}\"", mIndex, mJointRecord.name);
    return false;
  }
}

bool LinkBuilder::build(SArticulation &articulation) const {
  auto pxArticulation = articulation.mPxArticulation;
  auto &links = articulation.mLinks;
  auto &joints = articulation.mJoints;

  // create link
  physx_id_t linkId = mArticulationBuilder->getScene()->mLinkIdGenerator.next();
  PxArticulationLink *pxLink = pxArticulation->createLink(
      mParent >= 0 ? links[mParent]->getPxArticulationLink() : nullptr, {{0, 0, 0}, PxIdentity});

  std::vector<PxShape *> shapes;
  std::vector<PxReal> densities;
  buildShapes(shapes, densities);

  std::vector<physx_id_t> renderIds;
  std::vector<Renderer::IPxrRigidbody *> renderBodies;
  buildVisuals(renderBodies, renderIds);
  for (auto body : renderBodies) {
    body->setSegmentationId(linkId);
  }

  PxFilterData data;
  data.word0 = mCollisionGroup.w0;
  data.word1 = mCollisionGroup.w1;
  data.word2 = 0;
  data.word2 = 0;

  for (size_t i = 0; i < shapes.size(); ++i) {
    shapes[i]->setSimulationFilterData(data);
    pxLink->attachShape(*shapes[i]);
    shapes[i]->release(); // this shape is reference counted by link
  }
  if (shapes.size() && mUseDensity) {
    PxRigidBodyExt::updateMassAndInertia(*pxLink, densities.data(), shapes.size());
  } else {
    pxLink->setMass(mMass);
    pxLink->setCMassLocalPose(mCMassPose);
    pxLink->setMassSpaceInertiaTensor(mInertia);
  }

  // wrap link
  links[mIndex] = std::unique_ptr<SLink>(
      new SLink(pxLink, &articulation, linkId, mArticulationBuilder->getScene(), renderBodies));
  links[mIndex]->setName(mName);

  links[mIndex]->mCol1 = mCollisionGroup.w0;
  links[mIndex]->mCol2 = mCollisionGroup.w1;
  links[mIndex]->mIndex = mIndex;

  pxLink->userData = links[mIndex].get();

  // create and wrap joint
  auto joint = static_cast<PxArticulationJointReducedCoordinate *>(pxLink->getInboundJoint());
  std::unique_ptr<SJoint> j;
  if (joint) {
    joint->setJointType(mJointRecord.jointType);
    joint->setParentPose(mJointRecord.parentPose);
    joint->setChildPose(mJointRecord.childPose);
    j = std::unique_ptr<SJoint>(new SJoint(links[mParent].get(), links[mIndex].get(), joint));
    j->setLimits(mJointRecord.limits);
  } else {
    j = std::unique_ptr<SJoint>(new SJoint(nullptr, links[mIndex].get(), nullptr));
  }
  j->setName(mJointRecord.name);
  joints[mIndex] = std::move(j);

  return true;
}

ArticulationBuilder::ArticulationBuilder(SScene *scene) : mScene(scene) {}

LinkBuilder *ArticulationBuilder::createLinkBuilder(LinkBuilder *parent) {
  return createLinkBuilder(parent ? parent->mIndex : -1);
}

LinkBuilder *ArticulationBuilder::createLinkBuilder(int parentIdx) {
  mLinkBuilders.push_back(LinkBuilder(this, mLinkBuilders.size(), parentIdx));
  return &mLinkBuilders.back();
}

std::string ArticulationBuilder::summary() const {
  std::stringstream ss;
  ss << "======= Link Summary =======" << std::endl;
  for (auto &b : mLinkBuilders) {
    ss << b.summary() << std::endl;
  }
  ss << "======= Joint summary =======" << std::endl;
  return ss.str();
}

SArticulation *ArticulationBuilder::build(bool fixBase) const {
  // find tree root
  int root = -1;
  for (auto &b : mLinkBuilders) {
    if (b.mParent < 0) {
      if (root >= 0) {
        spdlog::error("Failed to build articulation: multiple roots");
        return nullptr;
      }
      root = b.mIndex;
    }
  }

  // root -> children
  std::vector<std::vector<int>> childMap(mLinkBuilders.size());
  for (auto &b : mLinkBuilders) {
    if (b.mParent >= 0) {
      childMap[b.mParent].push_back(b.mIndex);
    }
  }

  // tree traversal
  std::vector<int> visited(mLinkBuilders.size());
  std::vector<int> stack = {root};
  std::vector<int> sorted;
  while (!stack.empty()) {
    int elem = stack.back();
    stack.pop_back();
    sorted.push_back(elem);
    if (visited[elem]) {
      spdlog::error("Failed to build articulation: kinematic loop");
      return nullptr;
    }
    visited[elem] = 1;
    for (int child : childMap[elem]) {
      stack.push_back(child);
    }
  }

  for (auto &builder : mLinkBuilders) {
    if (!builder.checkJointProperties()) {
      spdlog::error("Failed to build articulation: invalid joint");
      return nullptr;
    }
  }

  auto sArticulation = std::unique_ptr<SArticulation>(new SArticulation(mScene));
  sArticulation->mPxArticulation =
      mScene->mSimulation->mPhysicsSDK->createArticulationReducedCoordinate();
  sArticulation->mPxArticulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, fixBase);

  sArticulation->mLinks.resize(mLinkBuilders.size());
  sArticulation->mJoints.resize(mLinkBuilders.size());

  // sorted now is topologically sorted
  for (int i : sorted) {
    if (!mLinkBuilders[i].build(*sArticulation)) {
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
      auto pxLink = link->getPxArticulationLink();
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
      uint32_t start = dofStarts[result->mLinks[i]->getPxArticulationLink()->getLinkIndex()];
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
  }

  result->mCache = result->mPxArticulation->createCache();
  result->mPxArticulation->zeroCache(*result->mCache);

  return result;
}

} // namespace sapien
