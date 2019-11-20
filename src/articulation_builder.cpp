#include "articulation_builder.h"
#include "common.h"
#include "mesh_registry.h"
#include <cassert>
#include <numeric>

namespace sapien {
using namespace MeshUtil;

ArticulationBuilder::ArticulationBuilder(Simulation *simulation)
    : mSimulation(simulation), mPhysicsSDK(simulation->mPhysicsSDK),
      mCooking(simulation->mCooking), mRenderer(simulation->mRenderer) {
  mArticulation = mPhysicsSDK->createArticulationReducedCoordinate();
}

static std::string jointType2jointTypeString(PxArticulationJointType::Enum type) {
  switch (type) {
  case PxArticulationJointType::Enum::ePRISMATIC: {
    return "prismatic";
  }
  case PxArticulationJointType::Enum::eREVOLUTE: {
    return "revolute";
  }
  case PxArticulationJointType::Enum::eSPHERICAL: {
    return "spherical";
  }
  case PxArticulationJointType::Enum::eFIX: {
    return "fixed";
  }
  case PxArticulationJointType::Enum::eUNDEFINED: {
    return "undefined";
  }
  }
}

PxArticulationLink *ArticulationBuilder::addLink(PxArticulationLink *parent,
                                                 const PxTransform &pose, const std::string &name,
                                                 const std::string &jointName,
                                                 PxArticulationJointType::Enum jointType,
                                                 std::vector<std::array<float, 2>> const &limits,
                                                 PxTransform const &parentPose,
                                                 PxTransform const &childPose) {
  PxArticulationLink *newLink = mArticulation->createLink(parent, pose);
  newLink->setName(newNameFromString(name));
  if (!name.empty()) {
    if (mNamedLinks.find(name) != mNamedLinks.end()) {
      throw std::runtime_error("Duplicate link names are not allowed in an articulation.");
    }
    mNamedLinks[name] = newLink;
  }
  mLink2JointName[newLink] = jointName;
  if (parent && jointType != PxArticulationJointType::eUNDEFINED) {
    auto joint = static_cast<PxArticulationJointReducedCoordinate *>(newLink->getInboundJoint());
    joint->setJointType(jointType);
    switch (jointType) {
    case physx::PxArticulationJointType::eFIX:
      break;
    case physx::PxArticulationJointType::eREVOLUTE:
      assert(limits.size() == 1);
      if (limits[0][0] == -std::numeric_limits<float>::infinity() &&
          limits[0][1] == std::numeric_limits<float>::infinity()) {
        joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
      } else {
        joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
        joint->setLimit(PxArticulationAxis::eTWIST, limits[0][0], limits[0][1]);
      }
      break;
    case physx::PxArticulationJointType::ePRISMATIC:
      if (limits[0][0] == -std::numeric_limits<float>::infinity() &&
          limits[0][1] == std::numeric_limits<float>::infinity()) {
        joint->setMotion(PxArticulationAxis::eX, PxArticulationMotion::eFREE);
      } else {
        joint->setMotion(PxArticulationAxis::eX, PxArticulationMotion::eLIMITED);
        joint->setLimit(PxArticulationAxis::eX, limits[0][0], limits[0][1]);
      }
      break;
    default:
      std::cerr << "Unsupported joint provided, fall back to fixed joint" << std::endl;
    }
    joint->setParentPose(parentPose);
    joint->setChildPose(childPose);
  }
  return newLink;
}

void ArticulationBuilder::addBoxShapeToLink(PxArticulationLink &link, const PxTransform &pose,
                                            const PxVec3 &size, PxMaterial *material) {
  material = material ? material : mSimulation->mDefaultMaterial;
  PxShape *shape = mPhysicsSDK->createShape(PxBoxGeometry(size), *material, true);
  shape->setLocalPose(pose);
  link.attachShape(*shape);
}

void ArticulationBuilder::addCapsuleShapeToLink(PxArticulationLink &link, const PxTransform &pose,
                                                PxReal radius, PxReal length,
                                                PxMaterial *material) {
  material = material ? material : mSimulation->mDefaultMaterial;
  PxShape *shape = mPhysicsSDK->createShape(PxCapsuleGeometry(radius, length), *material, true);
  shape->setLocalPose(pose);
  link.attachShape(*shape);
}

void ArticulationBuilder::addSphereShapeToLink(PxArticulationLink &link, const PxTransform &pose,
                                               PxReal radius, PxMaterial *material) {
  material = material ? material : mSimulation->mDefaultMaterial;
  PxShape *shape = mPhysicsSDK->createShape(PxSphereGeometry(radius), *material, true);
  shape->setLocalPose(pose);
  link.attachShape(*shape);
}

void ArticulationBuilder::addConvexObjShapeToLink(PxArticulationLink &link,
                                                  const std::string &filename,
                                                  const PxTransform &pose, const PxVec3 &scale,
                                                  PxMaterial *material) {
  material = material ? material : mSimulation->mDefaultMaterial;
  PxConvexMesh *mesh = loadObjMesh(filename, mPhysicsSDK, mCooking);
  if (mesh) {
    PxShape *shape =
        mPhysicsSDK->createShape(PxConvexMeshGeometry(mesh, PxMeshScale(scale)), *material, true);
    shape->setLocalPose(pose);
    link.attachShape(*shape);
  }
}

void ArticulationBuilder::setLinkMassAndInertia(PxArticulationLink &link, PxReal mass,
                                                const PxTransform &cMassPose,
                                                const PxVec3 &inertia) {
  link.setMass(mass);
  link.setCMassLocalPose(cMassPose);
  link.setMassSpaceInertiaTensor(inertia);
}

void ArticulationBuilder::updateLinkMassAndInertia(PxArticulationLink &link, PxReal density) {
  PxRigidBodyExt::updateMassAndInertia(link, density);
}

//===== Visual Functions =====//

physx_id_t ArticulationBuilder::addBoxVisualToLink(PxArticulationLink &link,
                                                   const PxTransform &pose, const PxVec3 &size,
                                                   const PxVec3 &color, const std::string &name) {
  if (!mRenderer)
    return 0;
  if (mLink2LinkId.find(&link) == mLink2LinkId.end()) {
    mLink2LinkId[&link] = IDGenerator::LinkId()->next();
  }
  physx_id_t newId = IDGenerator::RenderId()->next();
  mRenderer->addRigidbody(newId, PxGeometryType::eBOX, size, color);
  mRenderer->setSegmentationId(newId, mLink2LinkId[&link]);
  mRenderIds.push_back(newId);
  mSimulation->mRenderId2InitialPose[newId] = pose;
  mSimulation->mRenderId2Actor[newId] = &link;
  mSimulation->mRenderId2VisualName[newId] = name;
  mSimulation->mLinkId2Actor[mLink2LinkId[&link]] = &link;
  mSimulation->mActor2LinkId[&link] = mLink2LinkId[&link];
  mVisualName2RenderId[name].push_back(newId);
  mLink2RenderId[&link].push_back(newId);
  return newId;
}

physx_id_t ArticulationBuilder::addCapsuleVisualToLink(PxArticulationLink &link,
                                                       const PxTransform &pose, PxReal radius,
                                                       PxReal length, const PxVec3 &color,
                                                       const std::string &name) {
  if (!mRenderer)
    return 0;
  if (mLink2LinkId.find(&link) == mLink2LinkId.end()) {
    mLink2LinkId[&link] = IDGenerator::LinkId()->next();
  }
  physx_id_t newId = IDGenerator::RenderId()->next();
  mRenderer->addRigidbody(newId, PxGeometryType::eCAPSULE, {length, radius, radius}, color);
  mRenderer->setSegmentationId(newId, mLink2LinkId[&link]);
  mRenderIds.push_back(newId);
  mSimulation->mRenderId2InitialPose[newId] = pose;
  mSimulation->mRenderId2Actor[newId] = &link;
  mSimulation->mLinkId2Actor[mLink2LinkId[&link]] = &link;
  mSimulation->mActor2LinkId[&link] = mLink2LinkId[&link];
  mSimulation->mRenderId2VisualName[newId] = name;
  mVisualName2RenderId[name].push_back(newId);
  mLink2RenderId[&link].push_back(newId);
  return newId;
}

physx_id_t ArticulationBuilder::addSphereVisualToLink(PxArticulationLink &link,
                                                      const PxTransform &pose, PxReal radius,
                                                      const PxVec3 &color,
                                                      const std::string &name) {
  if (!mRenderer)
    return 0;
  if (mLink2LinkId.find(&link) == mLink2LinkId.end()) {
    mLink2LinkId[&link] = IDGenerator::LinkId()->next();
  }
  physx_id_t newId = IDGenerator::RenderId()->next();
  mRenderer->addRigidbody(newId, PxGeometryType::eSPHERE, {radius, radius, radius}, color);
  mRenderer->setSegmentationId(newId, mLink2LinkId[&link]);
  mRenderIds.push_back(newId);
  mSimulation->mRenderId2InitialPose[newId] = pose;
  mSimulation->mRenderId2Actor[newId] = &link;
  mSimulation->mLinkId2Actor[mLink2LinkId[&link]] = &link;
  mSimulation->mActor2LinkId[&link] = mLink2LinkId[&link];
  mSimulation->mRenderId2VisualName[newId] = name;
  mVisualName2RenderId[name].push_back(newId);
  mLink2RenderId[&link].push_back(newId);
  return newId;
}

physx_id_t ArticulationBuilder::addObjVisualToLink(PxArticulationLink &link,
                                                   const std::string &filename,
                                                   const PxTransform &pose, const PxVec3 &scale,
                                                   const std::string &name) {
  if (!mRenderer)
    return 0;
  if (mLink2LinkId.find(&link) == mLink2LinkId.end()) {
    mLink2LinkId[&link] = IDGenerator::LinkId()->next();
  }
  physx_id_t newId = IDGenerator::RenderId()->next();
  mRenderer->addRigidbody(newId, filename, scale);
  mRenderer->setSegmentationId(newId, mLink2LinkId[&link]);
  mRenderIds.push_back(newId);
  mSimulation->mRenderId2InitialPose[newId] = pose;
  mSimulation->mRenderId2Actor[newId] = &link;
  mSimulation->mLinkId2Actor[mLink2LinkId[&link]] = &link;
  mSimulation->mActor2LinkId[&link] = mLink2LinkId[&link];
  mSimulation->mRenderId2VisualName[newId] = name;
  mVisualName2RenderId[name].push_back(newId);
  mLink2RenderId[&link].push_back(newId);
  return newId;
}

//======== Collision funcions =======//
void ArticulationBuilder::disableCollision(PxArticulationLink &link1, PxArticulationLink &link2) {
  disableCollisionPair.push_back({&link1, &link2});
}

ArticulationWrapper *ArticulationBuilder::build(bool fixBase, bool balanceForce) {
  mArticulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, fixBase);
  auto wrapper = std::make_unique<ArticulationWrapper>();
  for (auto &p : mLink2LinkId) {
    mSimulation->mLinkId2Articulation[p.second] = wrapper.get();
  }

  // Balance force for robot articulation, for ant or other articulation, balanceForce should be
  // false (default)
  wrapper->set_force_balance(balanceForce);

  // add articulation
  wrapper->articulation = mArticulation;

  // add it to scene first
  mSimulation->mScene->addArticulation(*mArticulation);

  size_t nLinks = mArticulation->getNbLinks();
  // PxArticulationLink *links[nLinks];
  std::vector<PxArticulationLink *> links(nLinks);
  mArticulation->getLinks(links.data(), nLinks);
  std::sort(links.begin(), links.end(),
            [](const auto &a, const auto &b) { return a->getLinkIndex() < b->getLinkIndex(); });

  // disable collision
  for (auto &cp : disableCollisionPair) {
    PxArticulationLink *l1 = cp[0];
    PxArticulationLink *l2 = cp[1];
    int n1 = l1->getNbShapes();
    int n2 = l2->getNbShapes();

    // no shape to collide
    if (n1 == 0 || n2 == 0) {
      continue;
    }

    // TODO: decide inclusive or exclusive
    int colGroup = mSimulation->collisionManager.NewExclusiveGroup();

    std::vector<PxShape *> buf1(n1);
    l1->getShapes(buf1.data(), n1);
    for (int i = 0; i < n1; ++i) {
      PxFilterData data = buf1[i]->getSimulationFilterData();
      CollisionGroupManager::addGroupToData(data, colGroup);
      buf1[i]->setSimulationFilterData(data);
    }

    std::vector<PxShape *> buf2(n2);
    l2->getShapes(buf2.data(), n2);
    for (int i = 0; i < n2; ++i) {
      PxFilterData data = buf2[i]->getSimulationFilterData();
      CollisionGroupManager::addGroupToData(data, colGroup);
      buf2[i]->setSimulationFilterData(data);
    }
  }


  // for (size_t i = 0; i < nLinks; ++i) {
  //   if (PxArticulationJointBase *joint = links[i]->getInboundJoint()) {
  //     PxArticulationLink &parentLink = joint->getParentArticulationLink();
  //     PxArticulationLink &currentLink = *links[i];

  //     int n1 = currentLink.getNbShapes();
  //     int n2 = parentLink.getNbShapes();

  //     // no collision shape
  //     if (n1 == 0 || n2 == 0) {
  //       continue;
  //     }

  //     int colGroup = mSimulation->collisionManager.NewExclusiveGroup();

  //     PxShape *buf1[n1];
  //     currentLink.getShapes(buf1, n1);
  //     for (int i = 0; i < n1; ++i) {
  //       PxFilterData data = buf1[i]->getSimulationFilterData();
  //       CollisionGroupManager::addGroupToData(data, colGroup);
  //       buf1[i]->setSimulationFilterData(data);
  //     }

  //     PxShape *buf2[n2];
  //     parentLink.getShapes(buf2, n2);
  //     for (int i = 0; i < n2; ++i) {
  //       PxFilterData data = buf2[i]->getSimulationFilterData();
  //       CollisionGroupManager::addGroupToData(data, colGroup);
  //       buf2[i]->setSimulationFilterData(data);
  //     }
  //   }
  // }

  uint32_t jointIdx = 0;
  std::vector<int> jointIndices;
  // cache basic joint info
  for (size_t i = 0; i < nLinks; ++i) {
    auto *joint = static_cast<PxArticulationJointReducedCoordinate *>(links[i]->getInboundJoint());
    if (joint) {
      jointIndices.push_back(jointIdx++);
      wrapper->jointNames.push_back(mLink2JointName[links[i]]);
      wrapper->jointDofs.push_back(links[i]->getInboundJointDof());
      wrapper->jointTypes.push_back(jointType2jointTypeString(joint->getJointType()));
      for (size_t k = 0; k < wrapper->jointDofs.back(); ++k) {
        wrapper->jointNamesDOF.push_back(wrapper->jointNames.back());
        wrapper->activeJoints.push_back(joint);
      }

      switch (joint->getJointType()) {
      case physx::PxArticulationJointType::eREVOLUTE: {
        auto motion = joint->getMotion(PxArticulationAxis::eTWIST);
        wrapper->jointAxises.push_back(PxArticulationAxis::eTWIST);
        if (motion == PxArticulationMotion::eFREE) {
          wrapper->jointLimits.push_back(
              {-std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()});
        } else {
          PxReal low, high;
          joint->getLimit(PxArticulationAxis::eTWIST, low, high);
          joint->setFrictionCoefficient(0);
          wrapper->jointLimits.push_back({low, high});
        }
        break;
      }
      case physx::PxArticulationJointType::ePRISMATIC: {
        auto motion = joint->getMotion(PxArticulationAxis::eX);
        wrapper->jointAxises.push_back(PxArticulationAxis::eX);
        if (motion == PxArticulationMotion::eFREE) {
          wrapper->jointLimits.push_back(
              {-std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()});
        } else {
          PxReal low, high;
          joint->getLimit(PxArticulationAxis::eX, low, high);
          joint->setFrictionCoefficient(0);
          wrapper->jointLimits.push_back({low, high});
        }
        break;
      }
      case physx::PxArticulationJointType::eFIX:
        break;
      default:
        throw std::runtime_error("Unsupported joint\n");
      }
    } else {
      jointIndices.push_back(-1);
    }
  }

  // reference cache after adding to scene
  wrapper->cache = mArticulation->createCache();
  wrapper->articulation->copyInternalStateToCache(*wrapper->cache, PxArticulationCache::eALL);
  wrapper->articulation->setName("articulation");

  // Add links to the wrapper
  wrapper->linkName2Link = mNamedLinks;
  for (const auto link : links) {
    wrapper->links.push_back(link);
    wrapper->linkMasses.push_back(link->getMass());
    wrapper->linkInertial.push_back(link->getMassSpaceInertiaTensor());
    if (mLink2LinkId.find(link) != mLink2LinkId.end()) {
      wrapper->linkSegmentationIds.push_back(mLink2LinkId[link]);
    } else {
      wrapper->linkSegmentationIds.push_back(0);
    }
  }
  wrapper->link2JointIndices = jointIndices;

  ArticulationWrapper *wrapperPtr = wrapper.get();
  mSimulation->mDynamicArticulationWrappers.push_back(std::move(wrapper));

  return wrapperPtr;
}

} // namespace sapien
