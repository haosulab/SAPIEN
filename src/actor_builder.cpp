#include "actor_builder.h"
#include "mesh_registry.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

namespace sapien {
using namespace MeshUtil;

void ActorBuilder::addConvexShapeFromObj(const std::string &filename, const PxTransform &pose,
                                           const PxVec3 &scale, PxMaterial *material,
                                           PxReal density) {
  material = material ? material : mSimulation->mDefaultMaterial;
  PxConvexMesh *mesh = loadObjMesh(filename, mPhysicsSDK, mCooking);
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

void ActorBuilder::addPrimitiveShape(physx::PxGeometryType::Enum type, physx::PxTransform pose,
                                       physx::PxVec3 scale, PxMaterial *material, PxReal density) {
  if (!material) {
    material = mSimulation->mDefaultMaterial;
  }

  switch (type) {
  case physx::PxGeometryType::ePLANE: {
    PxShape *shape = mPhysicsSDK->createShape(PxPlaneGeometry(), *material);
    if (!shape) {
      std::cerr << "create shape failed!" << std::endl;
      exit(1);
    }
    shape->setLocalPose(pose);
    physx_id_t newId = IDGenerator::instance()->next();
    mRenderer->addRigidbody(newId, type, scale);

    mRenderIds.push_back(newId);
    mShapes.push_back(shape);
    mDensities.push_back(density);
    mCount++;
    break;
  }
  default:
    std::cerr << "create shape failed: Unsupported primitive type" << std::endl;
    break;
  }
}

physx_id_t ActorBuilder::addBoxVisual(const PxTransform &pose, const PxVec3 &size) {
  physx_id_t newId = IDGenerator::instance()->next();
  mRenderIds.push_back(newId);
  mRenderer->addRigidbody(newId, PxGeometryType::eBOX,
                          size); // TODO: check if default box size is 1 or 2
  mSimulation->mRenderId2InitialPose[newId] = pose;
  return newId;
}

physx_id_t ActorBuilder::addCapsuleVisual(const PxTransform &pose, PxReal radius, PxReal length) {
  physx_id_t newId = IDGenerator::instance()->next();
  mRenderIds.push_back(newId);
  mRenderer->addRigidbody(newId, PxGeometryType::eCAPSULE,
                          {length, radius, radius}); // TODO: check default orientation
  mSimulation->mRenderId2InitialPose[newId] = pose;
  return newId;
}

physx_id_t ActorBuilder::addSphereVisual(const PxTransform &pose, PxReal radius) {
  physx_id_t newId = IDGenerator::instance()->next();
  mRenderIds.push_back(newId);
  mRenderer->addRigidbody(newId, PxGeometryType::eSPHERE, {radius, radius, radius});
  mSimulation->mRenderId2InitialPose[newId] = pose;
  return newId;
}

physx_id_t ActorBuilder::addObjVisual(const std::string &filename, const PxTransform &pose,
                                  const PxVec3 &scale) {
  physx_id_t newId = IDGenerator::instance()->next();
  mRenderIds.push_back(newId);
  mRenderer->addRigidbody(newId, filename, scale);
  mSimulation->mRenderId2InitialPose[newId] = pose;
  return newId;
}
PxRigidActor *ActorBuilder::build(bool isStatic, bool isKinematic, bool addToScene) {
  PxRigidActor *actor;
  if (isStatic) {
    actor = mPhysicsSDK->createRigidStatic(PxTransform(PxIdentity));
    for (size_t i = 0; i < mShapes.size(); ++i) {
      actor->attachShape(*mShapes[i]);
    }
    for (size_t i = 0; i < mRenderIds.size(); ++i) {
      mSimulation->mRenderId2Parent[mRenderIds[i]] = actor;
    }
  } else {
    PxRigidDynamic *dActor = mPhysicsSDK->createRigidDynamic(PxTransform(PxIdentity));
    dActor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, isKinematic);
    actor = dActor;
    for (size_t i = 0; i < mShapes.size(); ++i) {
      actor->attachShape(*mShapes[i]);
    }
    for (size_t i = 0; i < mRenderIds.size(); ++i) {
      mSimulation->mRenderId2Parent[mRenderIds[i]] = actor;
    }
    PxRigidBodyExt::updateMassAndInertia(*dActor, mDensities.data(), mCount);
  }
  actor->setName("");
  if (addToScene) {
    mSimulation->mScene->addActor(*actor);
  }
  return actor;
}
} // namespace sapien
