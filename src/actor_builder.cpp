#include "actor_builder.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

static PxConvexMesh *loadObjMesh(const std::string &filename,
                                 PxPhysics *physics, PxCooking *cooking) {
  std::vector<PxVec3> vertices;

  std::ifstream f(filename);
  std::string line;

  std::string t;
  float a, b, c;
  while (std::getline(f, line)) {
    if (line[0] == '#') {
      continue;
    }
    std::istringstream iss(line);
    iss >> t;
    if (t == "v") {
      iss >> a >> b >> c;
      vertices.push_back({a, b, c});
    }
  }

  PxConvexMeshDesc convexDesc;
  convexDesc.points.count = vertices.size();
  convexDesc.points.stride = sizeof(PxVec3);
  convexDesc.points.data = vertices.data();
  convexDesc.flags =
      PxConvexFlag::eCOMPUTE_CONVEX | PxConvexFlag::eSHIFT_VERTICES;

  PxDefaultMemoryOutputStream buf;
  PxConvexMeshCookingResult::Enum result;
  if (!cooking->cookConvexMesh(convexDesc, buf, &result)) {
    std::cerr << "Unable to cook convex mesh." << std::endl;
    std::cerr << "Exiting..." << std::endl;
    exit(1);
  }
  PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
  PxConvexMesh *convexMesh = physics->createConvexMesh(input);
  return convexMesh;
}

void PxActorBuilder::addConvexShapeFromObj(
    const std::string &filename, const std::string &renderFilename,
    physx::PxTransform pose, PxMaterial *material, PxReal density) {
  if (!material) {
    material = mSimulation->mDefaultMaterial;
  }

  PxConvexMesh *mesh = loadObjMesh(filename, mPhysicsSDK, mCooking);
  PxShape *shape =
      mPhysicsSDK->createShape(PxConvexMeshGeometry(mesh), *material);
  if (!shape) {
    std::cerr << "create shape failed!" << std::endl;
    exit(1);
  }
  shape->setLocalPose(pose);
  physx_id_t newId = IDGenerator::instance()->next();
  mRenderer->addRigidbody(newId, renderFilename);

  mRenderIds.push_back(newId);
  mShapes.push_back(shape);
  mDensities.push_back(density);
  mCount++;
}

void PxActorBuilder::addPrimitiveShape(physx::PxGeometryType::Enum type,
                                  physx::PxTransform pose, physx::PxVec3 scale,
                                  PxMaterial *material, PxReal density) {
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
  if (mRenderIds.size() != mCount || mShapes.size() != mCount ||
      mDensities.size() != mCount) {
    std::cerr << "Invalid size!" << std::endl;
  }
#endif

  PxRigidActor *actor;
  if (isStatic) {
    actor = mPhysicsSDK->createRigidStatic(PxTransform(PxIdentity));
    for (auto shape : mShapes) {
      actor->attachShape(*shape);
    }
  } else {
    PxRigidDynamic *dActor =
        mPhysicsSDK->createRigidDynamic(PxTransform(PxIdentity));
    dActor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, isKinematic);
    actor = dActor;
    for (auto shape : mShapes) {
      actor->attachShape(*shape);
    }
    PxRigidBodyExt::updateMassAndInertia(*dActor, mDensities.data(), mCount);
  }

  mSimulation->mActor2Ids[actor] = mRenderIds;
  mSimulation->mScene->addActor(*actor);

  return actor;
}
