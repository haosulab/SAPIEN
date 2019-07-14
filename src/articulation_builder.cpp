#include "articulation_builder.h"

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

void PxArticulationBuilder::addPrimitiveShapeToLink(PxArticulationLink &link,
                                                    PxGeometryType::Enum type,
                                                    const PxTransform &pose,
                                                    PxMaterial *material) {
  if (!material) {
    material = mSimulation->mDefaultMaterial;
  }
  switch (type) {
  case PxGeometryType::eBOX: {
    PxShape *shape = mPhysicsSDK->createShape(PxBoxGeometry(PxVec3(1, 1, 1)), *material);
    if (!shape) {
      std::cerr << "create shape failed!" << std::endl;
      exit(1);
    }
    shape->setLocalPose(pose);
    break;
  }
  default: {
    std::cerr << "Only Box gemoetry is currently implemented" << std::endl;
    exit(1);
  }
  }
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
                                               const PxTransform &pose,
                                               const PxVec3 &scale
                                               ) {
  // generate new render id
  physx_id_t newId = IDGenerator::instance()->next();
  mRenderer->addRigidbody(newId, filename, scale);
  mSimulation->mRenderId2InitialPose[newId] = pose;
  mSimulation->mRenderId2Parent[newId] = &link;
}

PxArticulationInterface PxArticulationBuilder::build(bool fixBase) {
  mArticulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, fixBase);

  PxArticulationInterface interface;
  // add articulation
  interface.articulation = mArticulation;

  // add it to scene first
  mSimulation->mScene->addArticulation(*mArticulation);

  // reference cache after adding to scene
  interface.cache = mArticulation->createCache();

  // reference links
  size_t nLinks = mArticulation->getNbLinks();
  PxArticulationLink *links[nLinks];
  mArticulation->getLinks(links, nLinks);
  interface.links = std::vector<PxArticulationLink *>(links, links + nLinks);

  // reference named links
  interface.namedLinks = namedLinks;

  interface.dofStarts = std::vector<PxU32>(nLinks);

  for (PxU32 i = 1; i < nLinks; ++i) {
    PxU32 llIndex = links[i]->getLinkIndex();
    PxU32 dofs = links[i]->getInboundJointDof();
    printf("%ld %d %d\n", nLinks, llIndex, dofs);

    interface.dofStarts[llIndex] = dofs;
  }

  PxU32 count = 0;
  for (PxU32 i = 1; i < nLinks; ++i) {
    PxU32 dofs = interface.dofStarts[i];
    interface.dofStarts[i] = count;
    count += dofs;
  }

  return interface;
}
