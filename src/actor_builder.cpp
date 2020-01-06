#include "actor_builder.h"
#include "sapien_actor.h"
#include "sapien_scene.h"
#include "simulation.h"
#include <spdlog/spdlog.h>

namespace sapien {

Simulation *ActorBuilder::getSimulation() { return mScene->mSimulation; }

ActorBuilder::ActorBuilder(SScene *scene) : mScene(scene) {}

void ActorBuilder::addConvexShapeFromObj(const std::string &filename, const PxTransform &pose,
                                         const PxVec3 &scale, PxMaterial *material,
                                         PxReal density) {
  material = getSimulation()->mDefaultMaterial;
  PxConvexMesh *mesh = getSimulation()->getMeshManager().loadMesh(filename);

  if (!mesh) {
    spdlog::error("Failed to load convex shape for actor");
    return;
  }
  PxShape *shape = getSimulation()->mPhysicsSDK->createShape(
      PxConvexMeshGeometry(mesh, PxMeshScale(scale)), *material, true);

  if (!shape) {
    spdlog::critical("Failed to create shape");
    throw std::runtime_error("Failed to create shape");
  }

  shape->setLocalPose(pose);
  mShapes.push_back(shape);
  mDensities.push_back(density);
  mCount++;
}

void ActorBuilder::addMultipleConvexShapesFromObj(const std::string &filename,
                                                  const PxTransform &pose, const PxVec3 &scale,
                                                  PxMaterial *material, PxReal density) {
  material = material ? material : getSimulation()->mDefaultMaterial;
  auto meshes = getSimulation()->getMeshManager().loadMeshGroup(filename);
  spdlog::info("{} meshes loaded from {}", meshes.size(), filename);

  for (auto mesh : meshes) {
    PxShape *shape = getSimulation()->mPhysicsSDK->createShape(
        PxConvexMeshGeometry(mesh, PxMeshScale(scale)), *material, true);
    if (!shape) {
      spdlog::critical("Failed to create shapes");
      throw std::runtime_error("Failed to create shape");
    }
    shape->setLocalPose(pose);
    mShapes.push_back(shape);
    mDensities.push_back(density);
    mCount++;
  }
}

void ActorBuilder::addBoxShape(const PxTransform &pose, const PxVec3 &size, PxMaterial *material,
                               PxReal density) {
  material = material ? material : getSimulation()->mDefaultMaterial;
  PxShape *shape = getSimulation()->mPhysicsSDK->createShape(PxBoxGeometry(size), *material, true);
  shape->setLocalPose(pose);
  mShapes.push_back(shape);
  mDensities.push_back(density);
  mCount++;
}

void ActorBuilder::addCapsuleShape(const PxTransform &pose, PxReal radius, PxReal length,
                                   PxMaterial *material, PxReal density) {
  material = material ? material : getSimulation()->mDefaultMaterial;
  std::cerr
      << "Warning: PhysX only supports capsule primitive, converting cylinder into capsule..."
      << std::endl;
  PxShape *shape = getSimulation()->mPhysicsSDK->createShape(PxCapsuleGeometry(radius, length),
                                                             *material, true);
  shape->setLocalPose(pose);
  mShapes.push_back(shape);
  mDensities.push_back(density);
  mCount++;
}

void ActorBuilder::addSphereShape(const PxTransform &pose, PxReal radius, PxMaterial *material,
                                  PxReal density) {
  material = material ? material : getSimulation()->mDefaultMaterial;
  PxShape *shape =
      getSimulation()->mPhysicsSDK->createShape(PxSphereGeometry(radius), *material, true);
  shape->setLocalPose(pose);
  mShapes.push_back(shape);
  mDensities.push_back(density);
  mCount++;
}

physx_id_t ActorBuilder::addBoxVisual(const PxTransform &pose, const PxVec3 &size,
                                      const PxVec3 &color, std::string const &name) {
  if (!getSimulation()->getRenderer()) {
    spdlog::error("Failed to add visual: no renderer");
    return 0;
  }
  if (!mLinkId) {
    mLinkId = mScene->mLinkIdGenerator.next();
  }
  physx_id_t newId = mScene->mRenderIdGenerator.next();

  auto rScene = mScene->getRendererScene();
  auto body = rScene->addRigidbody(PxGeometryType::eBOX, size, color);
  body->setUniqueId(newId);
  body->setSegmentationId(mLinkId);
  body->setInitialPose(pose);
  mRenderBodies.push_back(body);

  mScene->mRenderId2VisualName[newId] = name;
  return newId;
}

physx_id_t ActorBuilder::addCapsuleVisual(const PxTransform &pose, PxReal radius, PxReal length,
                                          const PxVec3 &color, std::string const &name) {
  if (!getSimulation()->getRenderer()) {
    spdlog::error("Failed to add visual: no renderer");
    return 0;
  }
  if (!mLinkId) {
    mLinkId = mScene->mLinkIdGenerator.next();
  }

  physx_id_t newId = mScene->mRenderIdGenerator.next();

  auto rScene = mScene->getRendererScene();
  auto body = rScene->addRigidbody(PxGeometryType::eCAPSULE, {length, radius, radius}, color);
  body->setUniqueId(newId);
  body->setSegmentationId(mLinkId);
  body->setInitialPose(pose);
  mRenderBodies.push_back(body);

  mScene->mRenderId2VisualName[newId] = name;
  return newId;
}

physx_id_t ActorBuilder::addSphereVisual(const PxTransform &pose, PxReal radius,
                                         const PxVec3 &color, std::string const &name) {
  if (!getSimulation()->getRenderer()) {
    spdlog::error("Failed to add visual: no renderer");
    return 0;
  }
  if (!mLinkId) {
    mLinkId = mScene->mLinkIdGenerator.next();
  }

  physx_id_t newId = mScene->mRenderIdGenerator.next();

  auto rScene = mScene->getRendererScene();
  auto body = rScene->addRigidbody(PxGeometryType::eSPHERE, {radius, radius, radius}, color);
  body->setUniqueId(newId);
  body->setSegmentationId(mLinkId);
  body->setInitialPose(pose);
  mRenderBodies.push_back(body);

  mScene->mRenderId2VisualName[newId] = name;
  return newId;
}

physx_id_t ActorBuilder::addObjVisual(const std::string &filename, const PxTransform &pose,
                                      const PxVec3 &scale, std::string const &name) {
  if (!getSimulation()->getRenderer()) {
    spdlog::error("Failed to add visual: no renderer");
    return 0;
  }
  if (!mLinkId) {
    mLinkId = mScene->mLinkIdGenerator.next();
  }

  physx_id_t newId = mScene->mRenderIdGenerator.next();

  auto rScene = mScene->getRendererScene();
  auto body = rScene->addRigidbody(filename, scale);
  body->setUniqueId(newId);
  body->setSegmentationId(mLinkId);
  body->setInitialPose(pose);
  mRenderBodies.push_back(body);

  mScene->mRenderId2VisualName[newId] = name;
  return newId;
}

SActor *ActorBuilder::build(bool isStatic, bool isKinematic, std::string const &name) {
  PxRigidActor *actor = nullptr;
  if (isStatic) {
    actor = getSimulation()->mPhysicsSDK->createRigidStatic(PxTransform(PxIdentity));
    for (size_t i = 0; i < mShapes.size(); ++i) {
      actor->attachShape(*mShapes[i]);
    }
  } else {
    PxRigidDynamic *dActor =
        getSimulation()->mPhysicsSDK->createRigidDynamic(PxTransform(PxIdentity));
    dActor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, isKinematic);
    actor = dActor;
    for (size_t i = 0; i < mShapes.size(); ++i) {
      actor->attachShape(*mShapes[i]);
    }
    if (mCount) {
      PxRigidBodyExt::updateMassAndInertia(*dActor, mDensities.data(), mCount);
    }
  }

  auto sActor = std::unique_ptr<SActor>(new SActor(actor, mLinkId, mScene, mRenderBodies));
  sActor->setName(name);
  sActor->getPxActor();

  auto result = sActor.get();
  mScene->addActor(std::move(sActor));

  return result;
}

} // namespace sapien
