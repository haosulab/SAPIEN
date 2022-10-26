#include "sapien/actor_builder.h"
#include "sapien/sapien_actor.h"
#include "sapien/sapien_scene.h"
#include "sapien/simulation.h"
#include <spdlog/spdlog.h>

namespace sapien {

ActorBuilder::ActorBuilder(SScene *scene) : mScene(scene) {}

std::shared_ptr<ActorBuilder> ActorBuilder::removeAllShapes() {
  mShapeRecord.clear();
  return shared_from_this();
}
std::shared_ptr<ActorBuilder> ActorBuilder::removeAllVisuals() {
  mVisualRecord.clear();
  return shared_from_this();
}
int ActorBuilder::getShapeCount() const { return mShapeRecord.size(); }
int ActorBuilder::getVisualCount() const { return mVisualRecord.size(); }
std::shared_ptr<ActorBuilder> ActorBuilder::removeShapeAt(uint32_t index) {
  if (index < mShapeRecord.size()) {
    mShapeRecord.erase(mShapeRecord.begin() + index);
  }
  return shared_from_this();
}
std::shared_ptr<ActorBuilder> ActorBuilder::removeVisualAt(uint32_t index) {
  if (index < mVisualRecord.size()) {
    mVisualRecord.erase(mVisualRecord.begin() + index);
  }
  return shared_from_this();
}

std::shared_ptr<ActorBuilder> ActorBuilder::addNonConvexShapeFromFile(
    const std::string &filename, const PxTransform &pose, const PxVec3 &scale,
    std::shared_ptr<SPhysicalMaterial> material, PxReal patchRadius, PxReal minPatchRadius,
    bool isTrigger) {
  ShapeRecord r;
  r.type = ShapeRecord::Type::NonConvexMesh;
  r.filename = filename;
  r.pose = pose;
  r.scale = scale;
  r.material = material;
  r.density = 0.f;
  r.patchRadius = patchRadius;
  r.minPatchRadius = minPatchRadius;
  r.isTrigger = isTrigger;

  mShapeRecord.push_back(r);

  return shared_from_this();
}

std::shared_ptr<ActorBuilder>
ActorBuilder::addConvexShapeFromFile(const std::string &filename, const PxTransform &pose,
                                     const PxVec3 &scale,
                                     std::shared_ptr<SPhysicalMaterial> material, PxReal density,
                                     PxReal patchRadius, PxReal minPatchRadius, bool isTrigger) {
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

  return shared_from_this();
}

std::shared_ptr<ActorBuilder> ActorBuilder::addMultipleConvexShapesFromFile(
    const std::string &filename, const PxTransform &pose, const PxVec3 &scale,
    std::shared_ptr<SPhysicalMaterial> material, PxReal density, PxReal patchRadius,
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

  return shared_from_this();
}

std::shared_ptr<ActorBuilder>
ActorBuilder::addBoxShape(const PxTransform &pose, const PxVec3 &halfSize,
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

  return shared_from_this();
}

std::shared_ptr<ActorBuilder>
ActorBuilder::addCapsuleShape(const PxTransform &pose, PxReal radius, PxReal halfLength,
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

  return shared_from_this();
}

std::shared_ptr<ActorBuilder>
ActorBuilder::addSphereShape(const PxTransform &pose, PxReal radius,
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

  return shared_from_this();
}

std::shared_ptr<ActorBuilder>
ActorBuilder::addBoxVisualWithMaterial(const PxTransform &pose, const PxVec3 &halfSize,
                                       std::shared_ptr<Renderer::IPxrMaterial> material,
                                       std::string const &name) {
  auto renderer = mScene->getSimulation()->getRenderer();
  if (!renderer) {
    return shared_from_this();
  }
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

  return shared_from_this();
}

std::shared_ptr<ActorBuilder> ActorBuilder::addBoxVisual(const PxTransform &pose,
                                                         const PxVec3 &size, const PxVec3 &color,
                                                         std::string const &name) {
  auto renderer = mScene->getSimulation()->getRenderer();
  if (renderer) {
    auto mat = renderer->createMaterial();
    mat->setBaseColor({color.x, color.y, color.z, 1.f});
    addBoxVisualWithMaterial(pose, size, mat, name);
  }
  return shared_from_this();
}

std::shared_ptr<ActorBuilder> ActorBuilder::addCapsuleVisualWithMaterial(
    const PxTransform &pose, PxReal radius, PxReal halfLength,
    std::shared_ptr<Renderer::IPxrMaterial> material, std::string const &name) {
  auto renderer = mScene->getSimulation()->getRenderer();
  if (!renderer) {
    return shared_from_this();
  }
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

  return shared_from_this();
}

std::shared_ptr<ActorBuilder> ActorBuilder::addCapsuleVisual(const PxTransform &pose,
                                                             PxReal radius, PxReal halfLength,
                                                             const PxVec3 &color,
                                                             std::string const &name) {
  auto renderer = mScene->getSimulation()->getRenderer();
  if (renderer) {
    auto mat = renderer ? renderer->createMaterial() : nullptr;
    mat->setBaseColor({color.x, color.y, color.z, 1.f});
    addCapsuleVisualWithMaterial(pose, radius, halfLength, mat, name);
  }

  return shared_from_this();
}

std::shared_ptr<ActorBuilder>
ActorBuilder::addSphereVisualWithMaterial(const PxTransform &pose, PxReal radius,
                                          std::shared_ptr<Renderer::IPxrMaterial> material,
                                          std::string const &name) {
  auto renderer = mScene->getSimulation()->getRenderer();
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

  return shared_from_this();
}

std::shared_ptr<ActorBuilder> ActorBuilder::addSphereVisual(const PxTransform &pose, PxReal radius,
                                                            const PxVec3 &color,
                                                            std::string const &name) {
  auto renderer = mScene->getSimulation()->getRenderer();
  if (renderer) {
    auto mat = renderer ? renderer->createMaterial() : nullptr;
    mat->setBaseColor({color.x, color.y, color.z, 1.f});
    addSphereVisualWithMaterial(pose, radius, mat, name);
  }

  return shared_from_this();
}

std::shared_ptr<ActorBuilder> ActorBuilder::addVisualFromFile(
    const std::string &filename, const PxTransform &pose, const PxVec3 &scale,
    std::shared_ptr<Renderer::IPxrMaterial> material, std::string const &name) {
  VisualRecord r;
  r.type = VisualRecord::Type::File;
  r.pose = pose;
  r.scale = scale;
  r.material = material;
  r.filename = filename;
  r.name = name;

  mVisualRecord.push_back(r);

  return shared_from_this();
}

std::shared_ptr<ActorBuilder> ActorBuilder::addVisualFromMeshWithMaterial(
    std::shared_ptr<Renderer::IRenderMesh> mesh, const PxTransform &pose, const PxVec3 &scale,
    std::shared_ptr<Renderer::IPxrMaterial> material, std::string const &name) {
  auto renderer = mScene->getSimulation()->getRenderer();
  if (!material) {
    material = mScene->getSimulation()->getRenderer()->createMaterial();
  }
  VisualRecord r;
  r.type = VisualRecord::Type::Mesh;
  r.pose = pose;
  r.scale = scale;
  r.material = material;
  r.mesh = mesh;
  r.name = name;
  mVisualRecord.push_back(r);

  return shared_from_this();
}

std::shared_ptr<ActorBuilder>
ActorBuilder::setMassAndInertia(PxReal mass, PxTransform const &cMassPose, PxVec3 const &inertia) {
  mUseDensity = false;
  mMass = mass;
  mCMassPose = cMassPose;
  mInertia = inertia;

  return shared_from_this();
}

std::shared_ptr<ActorBuilder> ActorBuilder::setScene(SScene *scene) {
  mScene = scene;
  return shared_from_this();
}

void ActorBuilder::buildShapes(std::vector<std::unique_ptr<SCollisionShape>> &shapes,
                               std::vector<PxReal> &densities) const {
  for (auto &r : mShapeRecord) {
    auto material = r.material ? r.material : mScene->getDefaultMaterial();

    switch (r.type) {
    case ShapeRecord::Type::NonConvexMesh: {
      PxTriangleMesh *mesh =
          mScene->getSimulation()->getMeshManager().loadNonConvexMesh(r.filename);
      if (!mesh) {
        spdlog::get("SAPIEN")->error("Failed to load non-convex mesh for actor");
        continue;
      }
      auto shape = mScene->getSimulation()->createCollisionShape(
          PxTriangleMeshGeometry(mesh, PxMeshScale(r.scale)), material);
      if (!shape) {
        throw std::runtime_error("Failed to create non-convex shape");
      }
      shape->setLocalPose(r.pose);
      shape->setTorsionalPatchRadius(r.patchRadius);
      shape->setMinTorsionalPatchRadius(r.minPatchRadius);
      if (r.isTrigger) {
        shape->setIsTrigger(true);
      }
      shapes.push_back(std::move(shape));
      densities.push_back(0);
      break;
    }

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
    Renderer::IPxrRigidbody *body{};
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
    case VisualRecord::Type::File:
      body = rScene->addRigidbody(r.filename, r.scale, r.material);
      break;
    case VisualRecord::Type::Mesh:
      body = rScene->addRigidbody(r.mesh, r.scale, r.material);
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

std::shared_ptr<ActorBuilder> ActorBuilder::setCollisionGroup(uint32_t g0, uint32_t g1, uint32_t g2, uint32_t g3) {
  mCollisionGroup.w0 = g0;
  mCollisionGroup.w1 = g1;
  mCollisionGroup.w2 = g2;
  mCollisionGroup.w3 = g3;

  return shared_from_this();
}

std::shared_ptr<ActorBuilder> ActorBuilder::addCollisionGroup(uint32_t g0, uint32_t g1, uint32_t g2, uint32_t g3) {
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
  return shared_from_this();
}

std::shared_ptr<ActorBuilder> ActorBuilder::resetCollisionGroup() {
  mCollisionGroup.w0 = 1;
  mCollisionGroup.w1 = 1;
  mCollisionGroup.w2 = 0;
  mCollisionGroup.w3 = 0;
  return shared_from_this();
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
    case PxGeometryType::eTRIANGLEMESH: {
      PxTriangleMeshGeometry geom;
      shape->getPxShape()->getTriangleMeshGeometry(geom);

      std::vector<PxVec3> vertices;
      std::vector<PxVec3> normals;
      std::vector<uint32_t> triangles;

      PxTriangleMesh *mesh = geom.triangleMesh;
      const PxVec3 *verts = mesh->getVertices();

      if (mesh->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES) {
        auto indices = static_cast<const PxU16 *>(mesh->getTriangles());
        for (PxU32 i = 0; i < mesh->getNbTriangles(); i++) {
          uint32_t i0 = indices[3 * i];
          uint32_t i1 = indices[3 * i + 1];
          uint32_t i2 = indices[3 * i + 2];
          vertices.push_back(verts[i0]);
          vertices.push_back(verts[i1]);
          vertices.push_back(verts[i2]);
          PxVec3 normal = (verts[i1] - verts[i0]).cross(verts[i2] - verts[i0]);
          normal.normalize();
          normals.push_back(normal);
          normals.push_back(normal);
          normals.push_back(normal);

          triangles.push_back(3 * i);
          triangles.push_back(3 * i + 1);
          triangles.push_back(3 * i + 2);
          triangles.push_back(3 * i);
          triangles.push_back(3 * i + 2);
          triangles.push_back(3 * i + 1);
        }
      } else {
        auto indices = static_cast<const PxU32 *>(mesh->getTriangles());
        for (PxU32 i = 0; i < mesh->getNbTriangles(); i++) {
          uint32_t i0 = indices[3 * i];
          uint32_t i1 = indices[3 * i + 1];
          uint32_t i2 = indices[3 * i + 2];
          vertices.push_back(verts[i0]);
          vertices.push_back(verts[i1]);
          vertices.push_back(verts[i2]);
          PxVec3 normal = (verts[i1] - verts[i0]).cross(verts[i2] - verts[i0]);
          normal.normalize();
          normals.push_back(normal);
          normals.push_back(normal);
          normals.push_back(normal);

          triangles.push_back(3 * i);
          triangles.push_back(3 * i + 1);
          triangles.push_back(3 * i + 2);
          triangles.push_back(3 * i);
          triangles.push_back(3 * i + 2);
          triangles.push_back(3 * i + 1);
        }
      }
      cBody = rendererScene->addRigidbody(vertices, normals, triangles, geom.scale.scale,
                                          PxVec3{1, 0, 0});
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
    bool zero = true;
    for (float density : densities) {
      if (density > 1e-8) {
        zero = false;
        break;
      }
    }
    if (zero && !isKinematic) {
      spdlog::get("SAPIEN")->warn(
          "All shapes have 0 density. This will result in unexpected mass and inertia.");
    }
    if (!isKinematic) {
      PxRigidBodyExt::updateMassAndInertia(*actor, densities.data(), shapes.size());
    }
  } else {
    if (mMass < 1e-8 || mInertia.x < 1e-8 || mInertia.y < 1e-8 || mInertia.z < 1e-8) {
      spdlog::get("SAPIEN")->info(
          "Mass or inertia contains very small number, this is not allowed. "
          "Mass will be set to 1e-6 and inertia will be set to 1e-8 for stability. Actor: {0}",
          name);
      actor->setMass(1e-6);
      actor->setMassSpaceInertiaTensor({1e-8, 1e-8, 1e-8});
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

  result->mBuilder = shared_from_this();
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

  result->mBuilder = shared_from_this();
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
