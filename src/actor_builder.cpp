#include "actor_builder.h"
#include "common.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

namespace sapien {

void ActorBuilder::addConvexShapeFromObj(const std::string &filename, const PxTransform &pose,
                                         const PxVec3 &scale, PxMaterial *material,
                                         PxReal density) {
  material = material ? material : mSimulation->mDefaultMaterial;
  PxConvexMesh *mesh = mSimulation->getMeshManager().loadMesh(filename);
  if (!mesh) {
    return;
  }
  PxShape *shape =
      mPhysicsSDK->createShape(PxConvexMeshGeometry(mesh, PxMeshScale(scale)), *material, true);
  if (!shape) {
    std::cerr << "create shape failed!" << std::endl;
    exit(1);
  }
  shape->setLocalPose(pose);
  mShapes.push_back(shape);
  mDensities.push_back(density);
  mCount++;
}

void ActorBuilder::addMultipleConvexShapesFromObj(const std::string &filename,
                                                  const PxTransform &pose, const PxVec3 &scale,
                                                  PxMaterial *material, PxReal density) {
  material = material ? material : mSimulation->mDefaultMaterial;
  auto meshes = mSimulation->getMeshManager().loadMeshGroup(filename);
  std::cout << meshes.size() << " meshes loaded." << std::endl;
  for (auto mesh : meshes) {
    PxShape *shape =
        mPhysicsSDK->createShape(PxConvexMeshGeometry(mesh, PxMeshScale(scale)), *material, true);
    if (!shape) {
      std::cerr << "create shape failed!" << std::endl;
      exit(1);
    }
    shape->setLocalPose(pose);
    mShapes.push_back(shape);
    mDensities.push_back(density);
    mCount++;
  }
}

void ActorBuilder::addBoxShape(const PxTransform &pose, const PxVec3 &size, PxMaterial *material,
                               PxReal density) {
  material = material ? material : mSimulation->mDefaultMaterial;
  PxShape *shape = mPhysicsSDK->createShape(PxBoxGeometry(size), *material, true);
  shape->setLocalPose(pose);
  mShapes.push_back(shape);
  mDensities.push_back(density);
  mCount++;
}

void ActorBuilder::addCapsuleShape(const PxTransform &pose, PxReal radius, PxReal length,
                                   PxMaterial *material, PxReal density) {
  material = material ? material : mSimulation->mDefaultMaterial;
  std::cerr
      << "Warning: PhysX only supports capsule primitive, converting cylinder into capsule..."
      << std::endl;
  PxShape *shape = mPhysicsSDK->createShape(PxCapsuleGeometry(radius, length), *material, true);
  shape->setLocalPose(pose);
  mShapes.push_back(shape);
  mDensities.push_back(density);
  mCount++;
}

void ActorBuilder::addSphereShape(const PxTransform &pose, PxReal radius, PxMaterial *material,
                                  PxReal density) {
  material = material ? material : mSimulation->mDefaultMaterial;
  PxShape *shape = mPhysicsSDK->createShape(PxSphereGeometry(radius), *material, true);
  shape->setLocalPose(pose);
  mShapes.push_back(shape);
  mDensities.push_back(density);
  mCount++;
}

physx_id_t ActorBuilder::addBoxVisual(const PxTransform &pose, const PxVec3 &size,
                                      const PxVec3 &color, std::string const &name) {
  if (!mRenderer)
    return 0;
  if (!mLinkId) {
    mLinkId = IDGenerator::LinkId()->next();
  }
  physx_id_t newId = IDGenerator::RenderId()->next();
  mRenderIds.push_back(newId);
  mRenderer->addRigidbody(newId, PxGeometryType::eBOX, size, color);
  mRenderer->setSegmentationId(newId, mLinkId);
  mSimulation->mRenderId2InitialPose[newId] = pose;
  mSimulation->mRenderId2VisualName[newId] = name;
  return newId;
}

physx_id_t ActorBuilder::addCapsuleVisual(const PxTransform &pose, PxReal radius, PxReal length,
                                          const PxVec3 &color, std::string const &name) {
  if (!mRenderer)
    return 0;
  if (!mLinkId) {
    mLinkId = IDGenerator::LinkId()->next();
  }
  physx_id_t newId = IDGenerator::RenderId()->next();
  mRenderIds.push_back(newId);
  mRenderer->addRigidbody(newId, PxGeometryType::eCAPSULE, {length, radius, radius}, color);
  mRenderer->setSegmentationId(newId, mLinkId);
  mSimulation->mRenderId2InitialPose[newId] = pose;
  mSimulation->mRenderId2VisualName[newId] = name;
  return newId;
}

physx_id_t ActorBuilder::addSphereVisual(const PxTransform &pose, PxReal radius,
                                         const PxVec3 &color, std::string const &name) {
  if (!mRenderer)
    return 0;
  if (!mLinkId) {
    mLinkId = IDGenerator::LinkId()->next();
  }
  physx_id_t newId = IDGenerator::RenderId()->next();
  mRenderIds.push_back(newId);
  mRenderer->addRigidbody(newId, PxGeometryType::eSPHERE, {radius, radius, radius}, color);
  mRenderer->setSegmentationId(newId, mLinkId);
  mSimulation->mRenderId2InitialPose[newId] = pose;
  mSimulation->mRenderId2VisualName[newId] = name;
  return newId;
}

physx_id_t ActorBuilder::addObjVisual(const std::string &filename, const PxTransform &pose,
                                      const PxVec3 &scale, std::string const &name) {
  if (!mRenderer)
    return 0;
  if (!mLinkId) {
    mLinkId = IDGenerator::LinkId()->next();
  }
  physx_id_t newId = IDGenerator::RenderId()->next();
  mRenderIds.push_back(newId);
  mRenderer->addRigidbody(newId, filename, scale);
  mRenderer->setSegmentationId(newId, mLinkId);
  mSimulation->mRenderId2InitialPose[newId] = pose;
  mSimulation->mRenderId2VisualName[newId] = name;
  return newId;
}

PxRigidActor *ActorBuilder::build(bool isStatic, bool isKinematic, std::string const &name,
                                  bool addToScene) {
  PxRigidActor *actor;
  if (isStatic) {
    actor = mPhysicsSDK->createRigidStatic(PxTransform(PxIdentity));
    for (size_t i = 0; i < mShapes.size(); ++i) {
      actor->attachShape(*mShapes[i]);
    }
    for (size_t i = 0; i < mRenderIds.size(); ++i) {
      mSimulation->mRenderId2Actor[mRenderIds[i]] = actor;
    }
    mSimulation->mLinkId2Actor[mLinkId] = actor;
    mSimulation->mActor2LinkId[actor] = mLinkId;
  } else {
    PxRigidDynamic *dActor = mPhysicsSDK->createRigidDynamic(PxTransform(PxIdentity));
    dActor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, isKinematic);
    actor = dActor;
    for (size_t i = 0; i < mShapes.size(); ++i) {
      actor->attachShape(*mShapes[i]);
    }
    for (size_t i = 0; i < mRenderIds.size(); ++i) {
      mSimulation->mRenderId2Actor[mRenderIds[i]] = actor;
    }
    mSimulation->mLinkId2Actor[mLinkId] = actor;
    mSimulation->mActor2LinkId[actor] = mLinkId;
    if (mCount) {
      PxRigidBodyExt::updateMassAndInertia(*dActor, mDensities.data(), mCount);
    }
  }
  actor->setName(newNameFromString(name));
  if (addToScene) {
    mSimulation->mScene->addActor(*actor);
  }
  return actor;
}
} // namespace sapien
