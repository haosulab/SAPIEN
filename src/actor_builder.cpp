#include "actor_builder.h"
#include "sapien_actor.h"
#include "sapien_scene.h"
#include "simulation.h"
#include <spdlog/spdlog.h>

namespace sapien {

ActorBuilder::ActorBuilder(SScene *scene) : mScene(scene) {}

void ActorBuilder::removeAllShapes() { mShapeRecord.clear(); }
void ActorBuilder::removeAllVisuals() { mVisualRecord.clear(); }
int ActorBuilder::getShapeCount() const { return mShapeRecord.size(); }
int ActorBuilder::getVisualCount() const { return mVisualRecord.size(); }
void ActorBuilder::removeShapeAt(uint32_t index) {
  if (index < mShapeRecord.size()) {
    mShapeRecord.erase(mShapeRecord.begin() + index);
  }
}
void ActorBuilder::removeVisualAt(uint32_t index) {
  if (index < mVisualRecord.size()) {
    mVisualRecord.erase(mVisualRecord.begin() + index);
  }
}

void ActorBuilder::addConvexShapeFromFile(const std::string &filename, const PxTransform &pose,
                                          const PxVec3 &scale,
                                          std::shared_ptr<SPhysicalMaterial> material,
                                          PxReal density, PxReal patchRadius,
                                          PxReal minPatchRadius, bool isTrigger) {
  ShapeRecord r;
  r.type = ShapeRecord::Type::SingleMesh;
  r.filename = filename;
  r.pose = pose;
  r.scale = scale;
  r.material = material;
  r.density = density;
  r.patchRadius = patchRadius;
  r.minPatchRadius = minPatchRadius;
  r.isTrigger = isTrigger;

  mShapeRecord.push_back(r);
}

void ActorBuilder::addMultipleConvexShapesFromFile(const std::string &filename,
                                                   const PxTransform &pose, const PxVec3 &scale,
                                                   std::shared_ptr<SPhysicalMaterial> material,
                                                   PxReal density, PxReal patchRadius,
                                                   PxReal minPatchRadius, bool isTrigger) {

  ShapeRecord r;
  r.type = ShapeRecord::Type::MultipleMeshes;
  r.filename = filename;
  r.pose = pose;
  r.scale = scale;
  r.material = material;
  r.density = density;
  r.patchRadius = patchRadius;
  r.minPatchRadius = minPatchRadius;
  r.isTrigger = isTrigger;

  mShapeRecord.push_back(r);
}

void ActorBuilder::addBoxShape(const PxTransform &pose, const PxVec3 &halfSize,
                               std::shared_ptr<SPhysicalMaterial> material, PxReal density,
                               PxReal patchRadius, PxReal minPatchRadius, bool isTrigger) {
  ShapeRecord r;
  r.type = ShapeRecord::Type::Box;
  r.pose = pose;
  r.scale = halfSize;
  r.material = material;
  r.density = density;
  r.patchRadius = patchRadius;
  r.minPatchRadius = minPatchRadius;
  r.isTrigger = isTrigger;

  mShapeRecord.push_back(r);
}

void ActorBuilder::addCapsuleShape(const PxTransform &pose, PxReal radius, PxReal halfLength,
                                   std::shared_ptr<SPhysicalMaterial> material, PxReal density,
                                   PxReal patchRadius, PxReal minPatchRadius, bool isTrigger) {
  ShapeRecord r;
  r.type = ShapeRecord::Type::Capsule;
  r.pose = pose;
  r.radius = radius;
  r.length = halfLength;
  r.material = material;
  r.density = density;
  r.patchRadius = patchRadius;
  r.minPatchRadius = minPatchRadius;
  r.isTrigger = isTrigger;

  mShapeRecord.push_back(r);
}

void ActorBuilder::addSphereShape(const PxTransform &pose, PxReal radius,
                                  std::shared_ptr<SPhysicalMaterial> material, PxReal density,
                                  PxReal patchRadius, PxReal minPatchRadius, bool isTrigger) {
  ShapeRecord r;
  r.type = ShapeRecord::Type::Sphere;
  r.pose = pose;
  r.radius = radius;
  r.material = material;
  r.density = density;
  r.patchRadius = patchRadius;
  r.minPatchRadius = minPatchRadius;
  r.isTrigger = isTrigger;

  mShapeRecord.push_back(r);
}

void ActorBuilder::addBoxVisualWithMaterial(const PxTransform &pose, const PxVec3 &halfSize,
                                            std::shared_ptr<Renderer::IPxrMaterial> material,
                                            std::string const &name) {
  if (!material) {
    material = mScene->getSimulation()->getRenderer()->createMaterial();
  }
  VisualRecord r;
  r.type = VisualRecord::Type::Box;
  r.pose = pose;
  r.scale = halfSize;
  r.material = material;
  r.name = name;

  mVisualRecord.push_back(r);
}

void ActorBuilder::addBoxVisual(const PxTransform &pose, const PxVec3 &size, const PxVec3 &color,
                                std::string const &name) {
  auto mat = mScene->getSimulation()->getRenderer()->createMaterial();
  mat->setBaseColor({color.x, color.y, color.z, 1.f});
  addBoxVisualWithMaterial(pose, size, mat, name);
}

void ActorBuilder::addCapsuleVisualWithMaterial(const PxTransform &pose, PxReal radius,
                                                PxReal halfLength,
                                                std::shared_ptr<Renderer::IPxrMaterial> material,
                                                std::string const &name) {
  if (!material) {
    material = mScene->getSimulation()->getRenderer()->createMaterial();
  }
  VisualRecord r;
  r.type = VisualRecord::Type::Capsule;
  r.pose = pose;
  r.radius = radius;
  r.length = halfLength;
  r.material = material;
  r.name = name;

  mVisualRecord.push_back(r);
}

void ActorBuilder::addCapsuleVisual(const PxTransform &pose, PxReal radius, PxReal halfLength,
                                    const PxVec3 &color, std::string const &name) {
  auto mat = mScene->getSimulation()->getRenderer()->createMaterial();
  mat->setBaseColor({color.x, color.y, color.z, 1.f});
  addCapsuleVisualWithMaterial(pose, radius, halfLength, mat, name);
}

void ActorBuilder::addSphereVisualWithMaterial(const PxTransform &pose, PxReal radius,
                                               std::shared_ptr<Renderer::IPxrMaterial> material,
                                               std::string const &name) {
  if (!material) {
    material = mScene->getSimulation()->getRenderer()->createMaterial();
  }
  VisualRecord r;
  r.type = VisualRecord::Type::Sphere;
  r.pose = pose;
  r.radius = radius;
  r.material = material;
  r.name = name;

  mVisualRecord.push_back(r);
}

void ActorBuilder::addSphereVisual(const PxTransform &pose, PxReal radius, const PxVec3 &color,
                                   std::string const &name) {
  auto mat = mScene->getSimulation()->getRenderer()->createMaterial();
  mat->setBaseColor({color.x, color.y, color.z, 1.f});
  addSphereVisualWithMaterial(pose, radius, mat, name);
}

void ActorBuilder::addVisualFromFile(const std::string &filename, const PxTransform &pose,
                                     const PxVec3 &scale, std::string const &name) {

  VisualRecord r;
  r.type = VisualRecord::Type::Mesh;
  r.pose = pose;
  r.scale = scale;
  r.filename = filename;
  r.name = name;

  mVisualRecord.push_back(r);
}

void ActorBuilder::setMassAndInertia(PxReal mass, PxTransform const &cMassPose,
                                     PxVec3 const &inertia) {
  mUseDensity = false;
  mMass = mass;
  mCMassPose = cMassPose;
  mInertia = inertia;
}

void ActorBuilder::buildShapes(std::vector<std::unique_ptr<SCollisionShape>> &shapes,
                               std::vector<PxReal> &densities) const {
  for (auto &r : mShapeRecord) {
    auto material = r.material ? r.material : mScene->getDefaultMaterial();

    switch (r.type) {
    case ShapeRecord::Type::SingleMesh: {
      PxConvexMesh *mesh = mScene->getSimulation()->getMeshManager().loadMesh(r.filename);
      if (!mesh) {
        spdlog::get("SAPIEN")->error("Failed to load convex mesh for actor");
        continue;
      }
      auto shape = mScene->getSimulation()->createCollisionShape(
          PxConvexMeshGeometry(mesh, PxMeshScale(r.scale)), material);
      shape->setContactOffset(mScene->mDefaultContactOffset);
      if (!shape) {
        spdlog::get("SAPIEN")->critical("Failed to create shape");
        throw std::runtime_error("Failed to create shape");
      }
      shape->setLocalPose(r.pose);
      shape->setTorsionalPatchRadius(r.patchRadius);
      shape->setMinTorsionalPatchRadius(r.minPatchRadius);
      if (r.isTrigger) {
        shape->setIsTrigger(true);
      }
      shapes.push_back(std::move(shape));
      densities.push_back(r.density);
      break;
    }

    case ShapeRecord::Type::MultipleMeshes: {
      auto meshes = mScene->getSimulation()->getMeshManager().loadMeshGroup(r.filename);
      for (auto mesh : meshes) {
        if (!mesh) {
          spdlog::get("SAPIEN")->error("Failed to load part of the convex mesh for actor");
          continue;
        }
        auto shape = mScene->getSimulation()->createCollisionShape(
            PxConvexMeshGeometry(mesh, PxMeshScale(r.scale)), material);
        shape->setContactOffset(mScene->mDefaultContactOffset);
        if (!shape) {
          spdlog::get("SAPIEN")->critical("Failed to create shape");
          throw std::runtime_error("Failed to create shape");
        }
        shape->setLocalPose(r.pose);
        shape->setTorsionalPatchRadius(r.patchRadius);
        shape->setMinTorsionalPatchRadius(r.minPatchRadius);
        if (r.isTrigger) {
          shape->setIsTrigger(true);
        }
        shapes.push_back(std::move(shape));
        densities.push_back(r.density);
      }
      break;
    }

    case ShapeRecord::Type::Box: {
      auto shape = mScene->getSimulation()->createCollisionShape(PxBoxGeometry(r.scale), material);
      shape->setContactOffset(mScene->mDefaultContactOffset);
      if (!shape) {
        spdlog::get("SAPIEN")->critical("Failed to build box with scale {}, {}, {}", r.scale.x,
                                        r.scale.y, r.scale.z);
        throw std::runtime_error("Failed to create shape");
      }
      shape->setLocalPose(r.pose);
      shape->setTorsionalPatchRadius(r.patchRadius);
      shape->setMinTorsionalPatchRadius(r.minPatchRadius);
      if (r.isTrigger) {
        shape->setIsTrigger(true);
      }
      shapes.push_back(std::move(shape));
      densities.push_back(r.density);
      break;
    }

    case ShapeRecord::Type::Capsule: {
      auto shape = mScene->getSimulation()->createCollisionShape(
          PxCapsuleGeometry(r.radius, r.length), material);
      shape->setContactOffset(mScene->mDefaultContactOffset);
      if (!shape) {
        spdlog::get("SAPIEN")->critical("Failed to build capsule with radius {}, length {}",
                                        r.radius, r.length);
        throw std::runtime_error("Failed to create shape");
      }
      shape->setLocalPose(r.pose);
      shape->setTorsionalPatchRadius(r.patchRadius);
      shape->setMinTorsionalPatchRadius(r.minPatchRadius);
      if (r.isTrigger) {
        shape->setIsTrigger(true);
      }
      shapes.push_back(std::move(shape));
      densities.push_back(r.density);
      break;
    }

    case ShapeRecord::Type::Sphere: {
      auto shape =
          mScene->getSimulation()->createCollisionShape(PxSphereGeometry(r.radius), material);
      shape->setContactOffset(mScene->mDefaultContactOffset);
      if (!shape) {
        spdlog::get("SAPIEN")->critical("Failed to build sphere with radius {}", r.radius);
        throw std::runtime_error("Failed to create shape");
      }
      shape->setLocalPose(r.pose);
      shape->setTorsionalPatchRadius(r.patchRadius);
      shape->setMinTorsionalPatchRadius(r.minPatchRadius);
      if (r.isTrigger) {
        shape->setIsTrigger(true);
      }
      shapes.push_back(std::move(shape));
      densities.push_back(r.density);
      break;
    }
    }
  }
}

void ActorBuilder::buildVisuals(std::vector<Renderer::IPxrRigidbody *> &renderBodies,
                                std::vector<physx_id_t> &renderIds) const {

  auto rScene = mScene->getRendererScene();
  if (!rScene) {
    return;
  }
  for (auto &r : mVisualRecord) {
    Renderer::IPxrRigidbody *body;
    switch (r.type) {
    case VisualRecord::Type::Box:
      body = rScene->addRigidbody(PxGeometryType::eBOX, r.scale, r.material);
      break;
    case VisualRecord::Type::Sphere:
      body = rScene->addRigidbody(PxGeometryType::eSPHERE, {r.radius, r.radius, r.radius},
                                  r.material);
      break;
    case VisualRecord::Type::Capsule:
      body = rScene->addRigidbody(PxGeometryType::eCAPSULE, {r.length, r.radius, r.radius},
                                  r.material);
      break;
    case VisualRecord::Type::Mesh:
      body = rScene->addRigidbody(r.filename, r.scale);
      break;
    }
    if (body) {
      physx_id_t newId = mScene->mRenderIdGenerator.next();

      renderIds.push_back(newId);
      body->setUniqueId(newId);
      body->setInitialPose(r.pose);
      renderBodies.push_back(body);

      body->setName(r.name);
    }
  }
}

void ActorBuilder::setCollisionGroup(uint32_t g0, uint32_t g1, uint32_t g2, uint32_t g3) {
  mCollisionGroup.w0 = g0;
  mCollisionGroup.w1 = g1;
  mCollisionGroup.w2 = g2;
  mCollisionGroup.w3 = g3;
}

void ActorBuilder::addCollisionGroup(uint32_t g0, uint32_t g1, uint32_t g2, uint32_t g3) {
  if (g0) {
    mCollisionGroup.w0 |= 1 << (g0 - 1);
  }
  if (g1) {
    mCollisionGroup.w1 |= 1 << (g1 - 1);
  }
  if (g2) {
    mCollisionGroup.w2 |= 1 << (g2 - 1);
  }
  if (g3) {
    mCollisionGroup.w2 |= 1 << (g3 - 1);
  }
}

void ActorBuilder::resetCollisionGroup() {
  mCollisionGroup.w0 = 1;
  mCollisionGroup.w1 = 1;
  mCollisionGroup.w2 = 0;
  mCollisionGroup.w3 = 0;
}

void ActorBuilder::buildCollisionVisuals(
    std::vector<Renderer::IPxrRigidbody *> &collisionBodies,
    std::vector<std::unique_ptr<SCollisionShape>> &shapes) const {
  auto rendererScene = mScene->getRendererScene();
  if (!rendererScene) {
    return;
  }
  for (auto &shape : shapes) {
    Renderer::IPxrRigidbody *cBody;
    switch (shape->getPxShape()->getGeometryType()) {
    case PxGeometryType::eBOX: {
      PxBoxGeometry geom;
      shape->getPxShape()->getBoxGeometry(geom);
      cBody = rendererScene->addRigidbody(PxGeometryType::eBOX, geom.halfExtents, PxVec3{0, 0, 1});
      break;
    }
    case PxGeometryType::eSPHERE: {
      PxSphereGeometry geom;
      shape->getPxShape()->getSphereGeometry(geom);
      cBody = rendererScene->addRigidbody(
          PxGeometryType::eSPHERE, {geom.radius, geom.radius, geom.radius}, PxVec3{0, 0, 1});
      break;
    }
    case PxGeometryType::eCAPSULE: {
      PxCapsuleGeometry geom;
      shape->getPxShape()->getCapsuleGeometry(geom);
      cBody = rendererScene->addRigidbody(
          PxGeometryType::eCAPSULE, {geom.halfHeight, geom.radius, geom.radius}, PxVec3{0, 0, 1});
      break;
    }
    case PxGeometryType::eCONVEXMESH: {
      PxConvexMeshGeometry geom;
      shape->getPxShape()->getConvexMeshGeometry(geom);

      std::vector<PxVec3> vertices;
      std::vector<PxVec3> normals;
      std::vector<uint32_t> triangles;

      PxConvexMesh *convexMesh = geom.convexMesh;
      const PxVec3 *convexVerts = convexMesh->getVertices();
      const PxU8 *indexBuffer = convexMesh->getIndexBuffer();
      PxU32 nbPolygons = convexMesh->getNbPolygons();
      PxU32 offset = 0;
      for (PxU32 i = 0; i < nbPolygons; i++) {
        PxHullPolygon face;
        convexMesh->getPolygonData(i, face);

        const PxU8 *faceIndices = indexBuffer + face.mIndexBase;
        for (PxU32 j = 0; j < face.mNbVerts; j++) {
          vertices.push_back(convexVerts[faceIndices[j]]);
          normals.push_back(PxVec3(face.mPlane[0], face.mPlane[1], face.mPlane[2]));
        }

        for (PxU32 j = 2; j < face.mNbVerts; j++) {
          triangles.push_back(offset);
          triangles.push_back(offset + j - 1);
          triangles.push_back(offset + j);
        }

        offset += face.mNbVerts;
      }
      cBody = rendererScene->addRigidbody(vertices, normals, triangles, geom.scale.scale,
                                          PxVec3{0, 1, 0});
      break;
    }
    default:
      spdlog::get("SAPIEN")->error(
          "Failed to create collision shape rendering: unrecognized geometry type.");
      continue;
    }

    if (cBody) {
      cBody->setInitialPose(shape->getLocalPose());
      cBody->setVisible(false);
      cBody->setRenderMode(1);
      collisionBodies.push_back(cBody);
    }
  }
}

SActor *ActorBuilder::build(bool isKinematic, std::string const &name) const {
  physx_id_t actorId = mScene->mActorIdGenerator.next();

  std::vector<std::unique_ptr<SCollisionShape>> shapes;
  std::vector<PxReal> densities;
  buildShapes(shapes, densities);

  std::vector<physx_id_t> renderIds;
  std::vector<Renderer::IPxrRigidbody *> renderBodies;
  buildVisuals(renderBodies, renderIds);
  for (auto body : renderBodies) {
    body->setSegmentationId(actorId);
  }

  std::vector<Renderer::IPxrRigidbody *> collisionBodies;
  buildCollisionVisuals(collisionBodies, shapes);
  for (auto body : collisionBodies) {
    body->setSegmentationId(actorId);
  }

  PxRigidDynamic *actor =
      mScene->getSimulation()->mPhysicsSDK->createRigidDynamic(PxTransform(PxIdentity));
  auto sActor =
      std::unique_ptr<SActor>(new SActor(actor, actorId, mScene, renderBodies, collisionBodies));

  actor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, isKinematic);
  for (size_t i = 0; i < shapes.size(); ++i) {
    shapes[i]->setCollisionGroups(mCollisionGroup.w0, mCollisionGroup.w1, mCollisionGroup.w2,
                                  mCollisionGroup.w3);
    sActor->attachShape(std::move(shapes[i]));
  }
  if (shapes.size() && mUseDensity) {
    for (float density : densities) {
      if (density < 1e-8) {
        throw std::runtime_error(
            "Failed to build actor: one collision shape density is too small");
      }
    }
    PxRigidBodyExt::updateMassAndInertia(*actor, densities.data(), shapes.size());
  } else {
    if (actor->getMass() < 1e-8 || actor->getMassSpaceInertiaTensor().x < 1e-8 ||
        actor->getMassSpaceInertiaTensor().y < 1e-8 ||
        actor->getMassSpaceInertiaTensor().z < 1e-8) {
      spdlog::get("SAPIEN")->warn("Actor mass or inertia contains 0. This is not allowed.");
      actor->setMass(1e-6);
      actor->setMassSpaceInertiaTensor({1e-6, 1e-6, 1e-6});
    } else {
      actor->setMass(mMass);
      actor->setCMassLocalPose(mCMassPose);
      actor->setMassSpaceInertiaTensor(mInertia);
    }
  }

  sActor->setName(name);

  sActor->mCol1 = mCollisionGroup.w0;
  sActor->mCol2 = mCollisionGroup.w1;
  sActor->mCol3 = mCollisionGroup.w2;

  actor->userData = sActor.get();

  actor->setSleepThreshold(mScene->mDefaultSleepThreshold);
  actor->setSolverIterationCounts(mScene->mDefaultSolverIterations,
                                  mScene->mDefaultSolverVelocityIterations);

  auto result = sActor.get();
  mScene->addActor(std::move(sActor));

  return result;
}

SActorStatic *ActorBuilder::buildStatic(std::string const &name) const {
  physx_id_t actorId = mScene->mActorIdGenerator.next();

  std::vector<std::unique_ptr<SCollisionShape>> shapes;
  std::vector<PxReal> densities;
  buildShapes(shapes, densities);

  std::vector<physx_id_t> renderIds;
  std::vector<Renderer::IPxrRigidbody *> renderBodies;
  buildVisuals(renderBodies, renderIds);
  for (auto body : renderBodies) {
    body->setSegmentationId(actorId);
  }

  std::vector<Renderer::IPxrRigidbody *> collisionBodies;
  buildCollisionVisuals(collisionBodies, shapes);
  for (auto body : collisionBodies) {
    body->setSegmentationId(actorId);
  }

  PxFilterData data;
  data.word0 = mCollisionGroup.w0;
  data.word1 = mCollisionGroup.w1;
  data.word2 = mCollisionGroup.w2;
  data.word3 = 0;

  PxRigidStatic *actor =
      mScene->getSimulation()->mPhysicsSDK->createRigidStatic(PxTransform(PxIdentity));
  auto sActor = std::unique_ptr<SActorStatic>(
      new SActorStatic(actor, actorId, mScene, renderBodies, collisionBodies));
  for (size_t i = 0; i < shapes.size(); ++i) {
    shapes[i]->setCollisionGroups(mCollisionGroup.w0, mCollisionGroup.w1, mCollisionGroup.w2,
                                  mCollisionGroup.w3);
    sActor->attachShape(std::move(shapes[i]));
  }

  sActor->setName(name);

  sActor->mCol1 = mCollisionGroup.w0;
  sActor->mCol2 = mCollisionGroup.w1;
  sActor->mCol3 = mCollisionGroup.w2;

  actor->userData = sActor.get();

  auto result = sActor.get();
  mScene->addActor(std::move(sActor));

  return result;
}

SActorStatic *ActorBuilder::buildGround(PxReal altitude, bool render,
                                        std::shared_ptr<SPhysicalMaterial> material,
                                        std::shared_ptr<Renderer::IPxrMaterial> renderMaterial,
                                        std::string const &name) {
  physx_id_t actorId = mScene->mActorIdGenerator.next();
  material = material ? material : mScene->mDefaultMaterial;

  auto shape = mScene->getSimulation()->createCollisionShape(PxPlaneGeometry(), material);
  auto pose = PxTransformFromPlaneEquation(PxPlane(0.f, 0.f, 1.f, -altitude));
  shape->setLocalPose(pose);
  shape->setCollisionGroups(mCollisionGroup.w0, mCollisionGroup.w1, mCollisionGroup.w2,
                            mCollisionGroup.w3);

  std::vector<Renderer::IPxrRigidbody *> renderBodies;
  if (render && mScene->getRendererScene()) {
    if (!renderMaterial) {
      renderMaterial = mScene->getSimulation()->getRenderer()->createMaterial();
    }
    auto body =
        mScene->mRendererScene->addRigidbody(PxGeometryType::ePLANE, {10, 10, 10}, renderMaterial);
    body->setInitialPose(pose);
    renderBodies.push_back(body);

    physx_id_t newId = mScene->mRenderIdGenerator.next();
    body->setSegmentationId(actorId);
    body->setUniqueId(newId);
  }

  PxRigidStatic *ground =
      mScene->getSimulation()->mPhysicsSDK->createRigidStatic(PxTransform(PxIdentity));
  auto sActor =
      std::unique_ptr<SActorStatic>(new SActorStatic(ground, actorId, mScene, renderBodies, {}));
  sActor->attachShape(std::move(shape));

  sActor->setName(name);
  sActor->mCol1 = mCollisionGroup.w0;
  sActor->mCol2 = mCollisionGroup.w1;
  sActor->mCol3 = mCollisionGroup.w2;

  ground->userData = sActor.get();

  auto result = sActor.get();
  mScene->addActor(std::move(sActor));

  return result;
}

} // namespace sapien
