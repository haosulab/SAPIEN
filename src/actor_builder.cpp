#include "actor_builder.h"
#include "mesh_registry.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

using namespace MeshUtil;

void PxActorBuilder::addConvexShapeFromObj(const std::string &filename,
                                           const std::string &renderFilename,
                                           physx::PxTransform pose, PxMaterial *material,
                                           PxReal density) {
  if (!material) {
    material = mSimulation->mDefaultMaterial;
  }

  PxConvexMesh *mesh = loadObjMesh(filename, mPhysicsSDK, mCooking);
  PxShape *shape = mPhysicsSDK->createShape(PxConvexMeshGeometry(mesh), *material);
  if (!shape) {
    std::cerr << "create shape failed!" << std::endl;
    exit(1);
  }
  shape->setLocalPose(pose);
  physx_id_t newId = IDGenerator::instance()->next();
  mRenderer->addRigidbody(newId, renderFilename, {1, 1, 1});

  mRenderIds.push_back(newId);
  mShapes.push_back(shape);
  mDensities.push_back(density);
  mCount++;
}

void PxActorBuilder::addPrimitiveShape(physx::PxGeometryType::Enum type, physx::PxTransform pose,
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

PxRigidActor *PxActorBuilder::build(bool isStatic, bool isKinematic) {
#ifdef _DEBUG
  if (mRenderIds.size() != mCount || mShapes.size() != mCount || mDensities.size() != mCount) {
    std::cerr << "Invalid size!" << std::endl;
  }
#endif

  PxRigidActor *actor;
  if (isStatic) {
    actor = mPhysicsSDK->createRigidStatic(PxTransform(PxIdentity));
    for (size_t i = 0; i < mShapes.size(); ++i) {
      actor->attachShape(*mShapes[i]);
      mSimulation->mRenderId2InitialPose[mRenderIds[i]] = mShapes[i]->getLocalPose();
      mSimulation->mRenderId2Parent[mRenderIds[i]] = actor;
    }
  } else {
    PxRigidDynamic *dActor = mPhysicsSDK->createRigidDynamic(PxTransform(PxIdentity));
    dActor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, isKinematic);
    actor = dActor;
    for (size_t i = 0; i < mShapes.size(); ++i) {
      actor->attachShape(*mShapes[i]);
      mSimulation->mRenderId2InitialPose[mRenderIds[i]] = mShapes[i]->getLocalPose();
      mSimulation->mRenderId2Parent[mRenderIds[i]] = actor;
    }
    PxRigidBodyExt::updateMassAndInertia(*dActor, mDensities.data(), mCount);
  }

  mSimulation->mScene->addActor(*actor);

  return actor;
}
