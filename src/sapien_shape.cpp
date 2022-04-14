#include "sapien/sapien_shape.h"
#include "sapien/sapien_material.h"
#include <array>
#include <stdexcept>

using namespace physx;

namespace sapien {

SCollisionShape::SCollisionShape(physx::PxShape *shape) : mPxShape(shape) {
  mPxShape->userData = this;
}

void SCollisionShape::setActor(SActorBase *actor) { mActor = actor; }
SActorBase *SCollisionShape::getActor() const { return mActor; }

SCollisionShape::~SCollisionShape() { mPxShape->release(); }

void SCollisionShape::setCollisionGroups(uint32_t group0, uint32_t group1, uint32_t group2,
                                         uint32_t group3) {
  mPxShape->setSimulationFilterData(PxFilterData(group0, group1, group2, group3));
}

std::array<uint32_t, 4> SCollisionShape::getCollisionGroups() const {
  auto data = mPxShape->getSimulationFilterData();
  return {data.word0, data.word1, data.word2, data.word3};
}

void SCollisionShape::setRestOffset(PxReal offset) { mPxShape->setRestOffset(offset); }
PxReal SCollisionShape::getRestOffset() const { return mPxShape->getRestOffset(); }

void SCollisionShape::setContactOffset(PxReal offset) { mPxShape->setContactOffset(offset); }
PxReal SCollisionShape::getContactOffset() const { return mPxShape->getContactOffset(); }

PxTransform SCollisionShape::getLocalPose() const { return mPxShape->getLocalPose(); }
void SCollisionShape::setLocalPose(PxTransform const &pose) { mPxShape->setLocalPose(pose); }

void SCollisionShape::setTorsionalPatchRadius(PxReal radius) {
  mPxShape->setTorsionalPatchRadius(radius);
}
PxReal SCollisionShape::getTorsionalPatchRadius() const {
  return mPxShape->getTorsionalPatchRadius();
}

void SCollisionShape::setMinTorsionalPatchRadius(PxReal radius) {
  mPxShape->setMinTorsionalPatchRadius(radius);
}
PxReal SCollisionShape::getMinTorsionalPatchRadius() const {
  return mPxShape->getMinTorsionalPatchRadius();
}

void SCollisionShape::setIsTrigger(bool trigger) {
  if (trigger) {
    mPxShape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
    mPxShape->setFlag(PxShapeFlag::eTRIGGER_SHAPE, true);
  } else {
    mPxShape->setFlag(PxShapeFlag::eTRIGGER_SHAPE, false);
    mPxShape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);
  }
}
bool SCollisionShape::isTrigger() const {
  return mPxShape->getFlags() & PxShapeFlag::eTRIGGER_SHAPE;
}

void SCollisionShape::setPhysicalMaterial(std::shared_ptr<SPhysicalMaterial> material) {
  auto mat = material->getPxMaterial();
  mPxShape->setMaterials(&mat, 1);
  mPhysicalMaterial = material;
}
std::shared_ptr<SPhysicalMaterial> SCollisionShape::getPhysicalMaterial() const {
  return mPhysicalMaterial;
}

std::string SCollisionShape::getType() const {
  switch (mPxShape->getGeometryType()) {
  case PxGeometryType::eBOX:
    return "box";
  case PxGeometryType::eSPHERE:
    return "sphere";
  case PxGeometryType::eCAPSULE:
    return "capsule";
  case PxGeometryType::ePLANE:
    return "plane";
  case PxGeometryType::eCONVEXMESH:
    return "convex_mesh";
  case PxGeometryType::eTRIANGLEMESH:
    return "nonconvex_mesh";
  default:
    throw std::runtime_error("unsupported shape type");
  }
  throw std::runtime_error("unsupported shape type");
}

std::unique_ptr<SGeometry> SCollisionShape::getGeometry() const {
  switch (mPxShape->getGeometryType()) {
  case PxGeometryType::eBOX: {
    PxBoxGeometry g;
    mPxShape->getBoxGeometry(g);
    auto sg = std::make_unique<SBoxGeometry>();
    sg->halfLengths = g.halfExtents;
    return sg;
  }
  case PxGeometryType::eSPHERE: {
    PxSphereGeometry g;
    mPxShape->getSphereGeometry(g);
    auto sg = std::make_unique<SSphereGeometry>();
    sg->radius = g.radius;
    return sg;
  }
  case PxGeometryType::eCAPSULE: {
    PxCapsuleGeometry g;
    mPxShape->getCapsuleGeometry(g);
    auto sg = std::make_unique<SCapsuleGeometry>();
    sg->radius = g.radius;
    sg->halfLength = g.halfHeight;
    return sg;
  }
  case PxGeometryType::ePLANE: {
    auto sg = std::make_unique<SPlaneGeometry>();
    return sg;
  }
  case PxGeometryType::eCONVEXMESH: {
    PxConvexMeshGeometry g;
    mPxShape->getConvexMeshGeometry(g);
    auto sg = std::make_unique<SConvexMeshGeometry>();
    sg->scale = g.scale.scale;
    sg->rotation = g.scale.rotation;

    // fill vertices
    sg->vertices.reserve(3 * g.convexMesh->getNbVertices());
    auto vertices = g.convexMesh->getVertices();
    for (uint32_t i = 0; i < g.convexMesh->getNbVertices(); ++i) {
      sg->vertices.push_back(vertices[i].x);
      sg->vertices.push_back(vertices[i].y);
      sg->vertices.push_back(vertices[i].z);
    }

    // fill indices
    sg->indices.reserve(3 * g.convexMesh->getNbPolygons());
    auto indices = g.convexMesh->getIndexBuffer();
    for (uint32_t i = 0; i < g.convexMesh->getNbPolygons(); ++i) {
      PxHullPolygon polygon;
      g.convexMesh->getPolygonData(i, polygon);
      for (int j = 0; j < int(polygon.mNbVerts) - 2; ++j) {
        sg->indices.push_back(indices[polygon.mIndexBase]);
        sg->indices.push_back(indices[polygon.mIndexBase + j + 1]);
        sg->indices.push_back(indices[polygon.mIndexBase + j + 2]);
      }
    }
    return sg;
  }
  case PxGeometryType::eTRIANGLEMESH: {
    PxTriangleMeshGeometry g;
    mPxShape->getTriangleMeshGeometry(g);
    auto sg = std::make_unique<SNonconvexMeshGeometry>();
    sg->scale = g.scale.scale;
    sg->rotation = g.scale.rotation;

    // fill vertices
    sg->vertices.reserve(3 * g.triangleMesh->getNbVertices());
    auto vertices = g.triangleMesh->getVertices();
    for (uint32_t i = 0; i < g.triangleMesh->getNbVertices(); ++i) {
      sg->vertices.push_back(vertices[i].x);
      sg->vertices.push_back(vertices[i].y);
      sg->vertices.push_back(vertices[i].z);
    }

    // fill indices
    sg->indices.reserve(3 * g.triangleMesh->getNbTriangles());

    if (g.triangleMesh->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES) {
      auto indices = static_cast<const uint16_t *>(g.triangleMesh->getTriangles());
      for (uint32_t i = 0; i < g.triangleMesh->getNbTriangles(); ++i) {
        sg->indices.push_back(indices[3 * i]);
        sg->indices.push_back(indices[3 * i + 1]);
        sg->indices.push_back(indices[3 * i + 2]);
      }
    } else {
      auto indices = static_cast<const uint32_t *>(g.triangleMesh->getTriangles());
      for (uint32_t i = 0; i < g.triangleMesh->getNbTriangles(); ++i) {
        sg->indices.push_back(indices[3 * i]);
        sg->indices.push_back(indices[3 * i + 1]);
        sg->indices.push_back(indices[3 * i + 2]);
      }
    }
    return sg;
  }

  default:
    throw std::runtime_error("unsupported shape type");
  }
  throw std::runtime_error("unsupported shape type");
}

} // namespace sapien
