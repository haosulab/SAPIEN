#include "articulation_builder.h"
#include "mesh_registry.h"

using namespace MeshUtil;

PxArticulationBuilder::PxArticulationBuilder(PxSimulation *simulation)
    : mSimulation(simulation), mPhysicsSDK(simulation->mPhysicsSDK),
      mCooking(simulation->mCooking), mRenderer(simulation->mRenderer) {
  mArticulation = mPhysicsSDK->createArticulationReducedCoordinate();
}

PxArticulationLink *PxArticulationBuilder::addLink(PxArticulationLink *parent,
                                                   const PxTransform &pose,
                                                   const std::string &name) {
  PxArticulationLink *newLink = mArticulation->createLink(parent, pose);
  if (!name.empty()) {
    if (namedLinks.find(name) != namedLinks.end()) {
      std::cerr << "Duplicated link name" << std::endl;
      exit(1);
    }
    namedLinks[name] = newLink;
  }
  return newLink;
}

void PxArticulationBuilder::addBoxShapeToLink(PxArticulationLink &link, const PxTransform &pose,
                                              const PxVec3 &size, PxMaterial *material) {
  material = material ? material : mSimulation->mDefaultMaterial;
  PxShape *shape = mPhysicsSDK->createShape(PxBoxGeometry(size), *material, true);
  shape->setLocalPose(pose);
  link.attachShape(*shape);
}

void PxArticulationBuilder::addCylinderShapeToLink(PxArticulationLink &link,
                                                   const PxTransform &pose, PxReal radius,
                                                   PxReal length, PxMaterial *material) {
  material = material ? material : mSimulation->mDefaultMaterial;
  std::cerr
      << "Warning: PhysX only supports capsule primitive, converting cylinder into capsule..."
      << std::endl;
  PxShape *shape = mPhysicsSDK->createShape(PxCapsuleGeometry(radius, length), *material, true);
  shape->setLocalPose(pose);
  link.attachShape(*shape);
}

void PxArticulationBuilder::addSphereShapeToLink(PxArticulationLink &link, const PxTransform &pose,
                                                 PxReal radius, PxMaterial *material) {
  material = material ? material : mSimulation->mDefaultMaterial;
  PxShape *shape = mPhysicsSDK->createShape(PxSphereGeometry(radius), *material, true);
  shape->setLocalPose(pose);
  link.attachShape(*shape);
}

void PxArticulationBuilder::addConvexObjShapeToLink(PxArticulationLink &link,
                                                    const std::string &filename,
                                                    const PxTransform &pose, const PxVec3 &scale,
                                                    PxMaterial *material) {
  material = material ? material : mSimulation->mDefaultMaterial;
  PxConvexMesh *mesh = loadObjMesh(filename, mPhysicsSDK, mCooking);
  PxShape *shape =
      mPhysicsSDK->createShape(PxConvexMeshGeometry(mesh, PxMeshScale(scale)), *material, true);
  shape->setLocalPose(pose);
  link.attachShape(*shape);
}

void PxArticulationBuilder::setLinkMassAndInertia(PxArticulationLink &link, PxReal mass,
                                                  const PxTransform &cMassPose,
                                                  const PxVec3 &inertia) {
  link.setMass(mass);
  link.setCMassLocalPose(cMassPose);
  link.setMassSpaceInertiaTensor(inertia);
}

void PxArticulationBuilder::updateLinkMassAndInertia(PxArticulationLink &link, PxReal density) {
  PxRigidBodyExt::updateMassAndInertia(link, density);
}

//===== Visual Functions =====//

void PxArticulationBuilder::addBoxVisualToLink(PxArticulationLink &link, const PxTransform &pose,
                                               const PxVec3 &size) {
  physx_id_t newId = IDGenerator::instance()->next();
  mRenderer->addRigidbody(newId, PxGeometryType::eBOX,
                          size); // TODO: check if default box size is 1 or 2
  mSimulation->mRenderId2InitialPose[newId] = pose;
  mSimulation->mRenderId2Parent[newId] = &link;
}

void PxArticulationBuilder::addCylinderVisualToLink(PxArticulationLink &link,
                                                    const PxTransform &pose, PxReal radius,
                                                    PxReal length) {
  physx_id_t newId = IDGenerator::instance()->next();
  mRenderer->addRigidbody(newId, PxGeometryType::eCAPSULE,
                          {length, radius, radius}); // TODO: check default orientation
  mSimulation->mRenderId2InitialPose[newId] = pose;
  mSimulation->mRenderId2Parent[newId] = &link;
}

void PxArticulationBuilder::addSphereVisualToLink(PxArticulationLink &link,
                                                  const PxTransform &pose, PxReal radius) {
  physx_id_t newId = IDGenerator::instance()->next();
  mRenderer->addRigidbody(newId, PxGeometryType::eSPHERE, {radius, radius, radius});
  mSimulation->mRenderId2InitialPose[newId] = pose;
  mSimulation->mRenderId2Parent[newId] = &link;
}

void PxArticulationBuilder::addObjVisualToLink(PxArticulationLink &link,
                                               const std::string &filename,
                                               const PxTransform &pose, const PxVec3 &scale) {
  // generate new render id
  physx_id_t newId = IDGenerator::instance()->next();
  mRenderer->addRigidbody(newId, filename, scale);
  mSimulation->mRenderId2InitialPose[newId] = pose;
  mSimulation->mRenderId2Parent[newId] = &link;
}

PxArticulationWrapper PxArticulationBuilder::build(bool fixBase) {
  mArticulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, fixBase);

  PxArticulationWrapper interface;
  // add articulation
  interface.articulation = mArticulation;

  // add it to scene first
  mSimulation->mScene->addArticulation(*mArticulation);

  size_t nLinks = mArticulation->getNbLinks();
  PxArticulationLink *links[nLinks];
  mArticulation->getLinks(links, nLinks);

  // ignore collision between parent and child
  for (size_t i = 0; i < nLinks; ++i) {
    if (PxArticulationJointBase *joint = links[i]->getInboundJoint()) {
      PxArticulationLink &parentLink = joint->getParentArticulationLink();
      PxArticulationLink &currentLink = *links[i];

      int n1 = currentLink.getNbShapes();
      int n2 = parentLink.getNbShapes();

      // no collision shape
      if (n1 == 0 || n2 == 0) {
        continue;
      }

      int colGroup = mSimulation->collisionManager.NewExclusiveGroup();

      PxShape *buf1[n1];
      currentLink.getShapes(buf1, n1);
      for (int i = 0; i < n1; ++i) {
        PxFilterData data = buf1[i]->getSimulationFilterData();
        CollisionGroupManager::addGroupToData(data, colGroup);
        buf1[i]->setSimulationFilterData(data);
      }

      PxShape *buf2[n2];
      parentLink.getShapes(buf2, n2);
      for (int i = 0; i < n2; ++i) {
        PxFilterData data = buf2[i]->getSimulationFilterData();
        CollisionGroupManager::addGroupToData(data, colGroup);
        buf2[i]->setSimulationFilterData(data);
      }
    }
  }

  // reference cache after adding to scene
  interface.cache = mArticulation->createCache();

  // reference links
  interface.links = std::vector<PxArticulationLink *>(links, links + nLinks);

  // reference named links
  interface.namedLinks = namedLinks;

  interface.dofStarts = std::vector<PxU32>(nLinks);

  for (PxU32 i = 1; i < nLinks; ++i) {
    PxU32 llIndex = links[i]->getLinkIndex();
    PxU32 dofs = links[i]->getInboundJointDof();
    // printf("%ld %d %d\n", nLinks, llIndex, dofs);

    interface.dofStarts[llIndex] = dofs;
  }

  PxU32 count = 0;
  for (PxU32 i = 1; i < nLinks; ++i) {
    PxU32 dofs = interface.dofStarts[i];
    interface.dofStarts[i] = count;
    count += dofs;
  }

  // initialize the interface
  interface.articulation->copyInternalStateToCache(*interface.cache, PxArticulationCache::eALL);

  interface.articulation->setName("articulation");

  return interface;
}
