#include "sapien/component/physx/collision_shape.h"
#include "sapien/component/physx/material.h"
#include "sapien/component/physx/mesh.h"
#include "sapien/component/physx/physx_default.h"
#include "sapien/component/physx/physx_system.h"
#include "sapien/component/physx/rigid_component.h"
#include "sapien/math/conversion.h"

using namespace physx;
namespace sapien::component {

std::shared_ptr<PhysxRigidBaseComponent> PhysxCollisionShape::getParent() const {
  if (!mParent) {
    return nullptr;
  }
  return std::static_pointer_cast<PhysxRigidBaseComponent>(mParent->shared_from_this());
}

void PhysxCollisionShape::setCollisionGroups(std::array<uint32_t, 4> groups) {
  getPxShape()->setSimulationFilterData(PxFilterData(groups[0], groups[1], groups[2], groups[3]));
}
std::array<uint32_t, 4> PhysxCollisionShape::getCollisionGroups() const {
  auto data = getPxShape()->getSimulationFilterData();
  return {data.word0, data.word1, data.word2, data.word3};
}

void PhysxCollisionShape::setRestOffset(float offset) { getPxShape()->setRestOffset(offset); }
float PhysxCollisionShape::getRestOffset() const { return getPxShape()->getRestOffset(); }

void PhysxCollisionShape::setContactOffset(float offset) {
  getPxShape()->setContactOffset(offset);
}
float PhysxCollisionShape::getContactOffset() const { return getPxShape()->getContactOffset(); }

void PhysxCollisionShape::setTorsionalPatchRadius(float radius) {
  getPxShape()->setTorsionalPatchRadius(radius);
}
float PhysxCollisionShape::getTorsionalPatchRadius() const {
  return getPxShape()->getTorsionalPatchRadius();
}

void PhysxCollisionShape::setMinTorsionalPatchRadius(float radius) {
  getPxShape()->setMinTorsionalPatchRadius(radius);
}
float PhysxCollisionShape::getMinTorsionalPatchRadius() const {
  return getPxShape()->getMinTorsionalPatchRadius();
}

void PhysxCollisionShape::setLocalPose(Pose const &pose) {
  getPxShape()->setLocalPose(PoseToPxTransform(pose));
  if (mPxShape->getActor()) {
    throw std::runtime_error(
        "failed to set shape local pose: this function should only be used befor "
        "the shape is attached to a rigid-body-type component");
  }
}
Pose PhysxCollisionShape::getLocalPose() const {
  return PxTransformToPose(getPxShape()->getLocalPose());
}

void PhysxCollisionShape::setIsTrigger(bool trigger) {
  if (trigger) {
    mPxShape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
    mPxShape->setFlag(PxShapeFlag::eTRIGGER_SHAPE, true);
  } else {
    mPxShape->setFlag(PxShapeFlag::eTRIGGER_SHAPE, false);
    mPxShape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);
  }
}
bool PhysxCollisionShape::isTrigger() const {
  return mPxShape->getFlags() & PxShapeFlag::eTRIGGER_SHAPE;
}

void PhysxCollisionShape::setIsSceneQuery(bool query) {
  mPxShape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, query);
}
bool PhysxCollisionShape::isSceneQuery() const {
  return mPxShape->getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE;
}

void PhysxCollisionShape::setPhysicalMaterial(std::shared_ptr<PhysxMaterial> material) {
  if (!material) {
    throw std::runtime_error("material must not be null");
  }
  auto mat = material->getPxMaterial();
  mPxShape->setMaterials(&mat, 1);
  mPhysicalMaterial = material;
}
std::shared_ptr<PhysxMaterial> PhysxCollisionShape::getPhysicalMaterial() const {
  return mPhysicalMaterial;
}

PhysxCollisionShape::~PhysxCollisionShape() {
  if (mPxShape) {
    mPxShape->userData = nullptr;
    mPxShape->release();
  }
}

//////////////////// collision shape constructor ////////////////////

PhysxCollisionShapePlane::PhysxCollisionShapePlane(std::shared_ptr<PhysxMaterial> material) {
  mEngine = PhysxEngine::Get();
  mPhysicalMaterial = material ? material : PhysxDefault::Get().getDefaultMaterial();
  mPxShape = mEngine->getPxPhysics()->createShape(PxPlaneGeometry(),
                                                  *getPhysicalMaterial()->getPxMaterial(), true);
  mPxShape->userData = this;
  setIsSceneQuery(true);
  setCollisionGroups({1, 1, 0, 0});
}

PhysxCollisionShapeBox::PhysxCollisionShapeBox(Vec3 halfLengths,
                                               std::shared_ptr<PhysxMaterial> material) {
  mEngine = PhysxEngine::Get();
  mPhysicalMaterial = material ? material : PhysxDefault::Get().getDefaultMaterial();
  mPxShape = mEngine->getPxPhysics()->createShape(PxBoxGeometry(Vec3ToPxVec3(halfLengths)),
                                                  *getPhysicalMaterial()->getPxMaterial(), true);
  mPxShape->userData = this;
  setIsSceneQuery(true);
  setCollisionGroups({1, 1, 0, 0});
}

PhysxCollisionShapeCapsule::PhysxCollisionShapeCapsule(float radius, float halfLength,
                                                       std::shared_ptr<PhysxMaterial> material) {
  mEngine = PhysxEngine::Get();
  mPhysicalMaterial = material ? material : PhysxDefault::Get().getDefaultMaterial();
  mPxShape = mEngine->getPxPhysics()->createShape(PxCapsuleGeometry(radius, halfLength),
                                                  *getPhysicalMaterial()->getPxMaterial(), true);
  mPxShape->userData = this;
  setIsSceneQuery(true);
  setCollisionGroups({1, 1, 0, 0});
}

PhysxCollisionShapeCylinder::PhysxCollisionShapeCylinder(float radius, float halfLength,
                                                         std::shared_ptr<PhysxMaterial> material) {
  mEngine = PhysxEngine::Get();
  mPhysicalMaterial = material ? material : PhysxDefault::Get().getDefaultMaterial();
  mMesh = PhysxConvexMesh::CreateCylinder();

  mRadius = radius;
  mHalfLength = halfLength;

  mPxShape = mEngine->getPxPhysics()->createShape(
      PxConvexMeshGeometry(mMesh->getPxMesh(), PxMeshScale({halfLength, radius, radius})),
      *getPhysicalMaterial()->getPxMaterial(), true);
  mPxShape->userData = this;
  setIsSceneQuery(true);
  setCollisionGroups({1, 1, 0, 0});
}

PhysxCollisionShapeSphere::PhysxCollisionShapeSphere(float radius,
                                                     std::shared_ptr<PhysxMaterial> material) {
  mEngine = PhysxEngine::Get();
  mPhysicalMaterial = material ? material : PhysxDefault::Get().getDefaultMaterial();
  mPxShape = mEngine->getPxPhysics()->createShape(PxSphereGeometry(radius),
                                                  *getPhysicalMaterial()->getPxMaterial(), true);
  mPxShape->userData = this;
  setIsSceneQuery(true);
  setCollisionGroups({1, 1, 0, 0});
}

PhysxCollisionShapeConvexMesh::PhysxCollisionShapeConvexMesh(
    std::shared_ptr<PhysxConvexMesh> mesh, Vec3 const &scale,
    std::shared_ptr<PhysxMaterial> material) {
  mEngine = PhysxEngine::Get();
  mPhysicalMaterial = material ? material : PhysxDefault::Get().getDefaultMaterial();
  mMesh = mesh;
  mPxShape = mEngine->getPxPhysics()->createShape(
      PxConvexMeshGeometry(mMesh->getPxMesh(), PxMeshScale(Vec3ToPxVec3(scale))),
      *getPhysicalMaterial()->getPxMaterial(), true);
  mPxShape->userData = this;

  auto aabb = computeAABB(getVertices());
  mLocalAABB = {aabb.lower * scale, aabb.upper * scale};
  setIsSceneQuery(true);
  setCollisionGroups({1, 1, 0, 0});
}

PhysxCollisionShapeConvexMesh::PhysxCollisionShapeConvexMesh(
    std::string const &filename, Vec3 const &scale, std::shared_ptr<PhysxMaterial> material) {
  mEngine = PhysxEngine::Get();
  mPhysicalMaterial = material ? material : PhysxDefault::Get().getDefaultMaterial();
  mMesh = MeshManager::Get()->loadConvexMesh(filename);
  mPxShape = mEngine->getPxPhysics()->createShape(
      PxConvexMeshGeometry(mMesh->getPxMesh(), PxMeshScale(Vec3ToPxVec3(scale))),
      *getPhysicalMaterial()->getPxMaterial(), true);
  mPxShape->userData = this;

  auto aabb = computeAABB(getVertices());
  mLocalAABB = {aabb.lower * scale, aabb.upper * scale};
  setIsSceneQuery(true);
  setCollisionGroups({1, 1, 0, 0});
}

PhysxCollisionShapeConvexMesh::PhysxCollisionShapeConvexMesh(
    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &vertices, Vec3 const &scale,
    std::shared_ptr<PhysxMaterial> material) {
  mEngine = PhysxEngine::Get();
  mPhysicalMaterial = material ? material : PhysxDefault::Get().getDefaultMaterial();
  mMesh = std::make_shared<PhysxConvexMesh>(vertices);
  mPxShape = mEngine->getPxPhysics()->createShape(
      PxConvexMeshGeometry(mMesh->getPxMesh(), PxMeshScale(Vec3ToPxVec3(scale))),
      *getPhysicalMaterial()->getPxMaterial(), true);

  mPxShape->userData = this;

  auto aabb = computeAABB(getVertices());
  mLocalAABB = {aabb.lower * scale, aabb.upper * scale};
  setIsSceneQuery(true);
  setCollisionGroups({1, 1, 0, 0});
}

std::vector<std::shared_ptr<PhysxCollisionShapeConvexMesh>>
PhysxCollisionShapeConvexMesh::LoadMultiple(std::string const &filename, Vec3 scale,
                                            std::shared_ptr<PhysxMaterial> material) {
  std::vector<std::shared_ptr<PhysxCollisionShapeConvexMesh>> result;

  auto meshes = MeshManager::Get()->loadConvexMeshGroup(filename);
  for (auto mesh : meshes) {
    result.push_back(std::make_shared<PhysxCollisionShapeConvexMesh>(mesh, scale, material));
  }
  return result;
}

PhysxCollisionShapeTriangleMesh::PhysxCollisionShapeTriangleMesh(
    std::string const &filename, Vec3 const &scale, std::shared_ptr<PhysxMaterial> material) {
  mEngine = PhysxEngine::Get();
  mPhysicalMaterial = material ? material : PhysxDefault::Get().getDefaultMaterial();
  mMesh = MeshManager::Get()->loadTriangleMesh(filename);
  mPxShape = mEngine->getPxPhysics()->createShape(
      PxTriangleMeshGeometry(mMesh->getPxMesh(), PxMeshScale(Vec3ToPxVec3(scale))),
      *getPhysicalMaterial()->getPxMaterial(), true);
  mPxShape->userData = this;

  auto aabb = computeAABB(getVertices());
  mLocalAABB = {aabb.lower * scale, aabb.upper * scale};
  setIsSceneQuery(true);
  setCollisionGroups({1, 1, 0, 0});
}

PhysxCollisionShapeTriangleMesh::PhysxCollisionShapeTriangleMesh(
    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &vertices,
    Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor> const &triangles,
    Vec3 const &scale, std::shared_ptr<PhysxMaterial> material) {
  mEngine = PhysxEngine::Get();
  mPhysicalMaterial = material ? material : PhysxDefault::Get().getDefaultMaterial();
  mMesh = std::make_shared<PhysxTriangleMesh>(vertices, triangles);
  mPxShape = mEngine->getPxPhysics()->createShape(
      PxTriangleMeshGeometry(mMesh->getPxMesh(), PxMeshScale(Vec3ToPxVec3(scale))),
      *getPhysicalMaterial()->getPxMaterial(), true);
  mPxShape->userData = this;

  auto aabb = computeAABB(getVertices());
  mLocalAABB = {aabb.lower * scale, aabb.upper * scale};
  setIsSceneQuery(true);
  setCollisionGroups({1, 1, 0, 0});
}

// internal use only
PhysxCollisionShapeTriangleMesh::PhysxCollisionShapeTriangleMesh(
    std::shared_ptr<PhysxTriangleMesh> mesh, Vec3 const &scale,
    std::shared_ptr<PhysxMaterial> material) {
  mEngine = PhysxEngine::Get();
  mPhysicalMaterial = material ? material : PhysxDefault::Get().getDefaultMaterial();
  mMesh = mesh;
  mPxShape = mEngine->getPxPhysics()->createShape(
      PxTriangleMeshGeometry(mMesh->getPxMesh(), PxMeshScale(Vec3ToPxVec3(scale))),
      *getPhysicalMaterial()->getPxMaterial(), true);
  mPxShape->userData = this;

  auto aabb = computeAABB(getVertices());
  mLocalAABB = {aabb.lower * scale, aabb.upper * scale};
  setIsSceneQuery(true);
  setCollisionGroups({1, 1, 0, 0});
}

//////////////////// end collision shape constructor ////////////////////

Vec3 PhysxCollisionShapeBox::getHalfLengths() const {
  auto &g = mPxShape->getGeometry();
  assert(g.getType() == PxGeometryType::eBOX);
  return PxVec3ToVec3(static_cast<PxBoxGeometry const &>(g).halfExtents);
}

float PhysxCollisionShapeCapsule::getRadius() const {
  auto &g = mPxShape->getGeometry();
  assert(g.getType() == PxGeometryType::eCAPSULE);
  return static_cast<PxCapsuleGeometry const &>(g).radius;
}

float PhysxCollisionShapeCapsule::getHalfLength() const {
  auto &g = mPxShape->getGeometry();
  assert(g.getType() == PxGeometryType::eCAPSULE);
  return static_cast<PxCapsuleGeometry const &>(g).halfHeight;
}

float PhysxCollisionShapeCylinder::getRadius() const { return mRadius; }
float PhysxCollisionShapeCylinder::getHalfLength() const { return mHalfLength; }

float PhysxCollisionShapeSphere::getRadius() const {
  auto &g = mPxShape->getGeometry();
  assert(g.getType() == PxGeometryType::eSPHERE);
  return static_cast<PxSphereGeometry const &>(g).radius;
}

Vec3 PhysxCollisionShapeConvexMesh::getScale() const {
  auto &g = mPxShape->getGeometry();
  assert(g.getType() == PxGeometryType::eCONVEXMESH);
  return PxVec3ToVec3(static_cast<PxConvexMeshGeometry const &>(g).scale.scale);
}

Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>
PhysxCollisionShapeConvexMesh::getVertices() const {
  auto &g_ = mPxShape->getGeometry();
  assert(g_.getType() == PxGeometryType::eCONVEXMESH);
  auto &g = static_cast<PxConvexMeshGeometry const &>(g_);

  std::vector<float> vertices;
  vertices.reserve(3 * g.convexMesh->getNbVertices());
  auto gv = g.convexMesh->getVertices();
  for (uint32_t i = 0; i < g.convexMesh->getNbVertices(); ++i) {
    vertices.push_back(gv[i].x);
    vertices.push_back(gv[i].y);
    vertices.push_back(gv[i].z);
  }
  return Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>>(
      vertices.data(), vertices.size() / 3, 3);
}

Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor>
PhysxCollisionShapeConvexMesh::getTriangles() const {
  auto &g_ = mPxShape->getGeometry();
  assert(g_.getType() == PxGeometryType::eCONVEXMESH);
  auto &g = static_cast<PxConvexMeshGeometry const &>(g_);

  std::vector<uint32_t> indices;
  auto gi = g.convexMesh->getIndexBuffer();
  for (uint32_t i = 0; i < g.convexMesh->getNbPolygons(); ++i) {
    PxHullPolygon polygon;
    g.convexMesh->getPolygonData(i, polygon);
    for (int j = 0; j < int(polygon.mNbVerts) - 2; ++j) {
      indices.push_back(gi[polygon.mIndexBase]);
      indices.push_back(gi[polygon.mIndexBase + j + 1]);
      indices.push_back(gi[polygon.mIndexBase + j + 2]);
    }
  }
  return Eigen::Map<Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor>>(
      indices.data(), indices.size() / 3, 3);
}

Vec3 PhysxCollisionShapeTriangleMesh::getScale() const {
  auto &g = mPxShape->getGeometry();
  assert(g.getType() == PxGeometryType::eTRIANGLEMESH);
  return PxVec3ToVec3(static_cast<PxTriangleMeshGeometry const &>(g).scale.scale);
}

Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>
PhysxCollisionShapeTriangleMesh::getVertices() const {
  auto &g_ = mPxShape->getGeometry();
  assert(g_.getType() == PxGeometryType::eTRIANGLEMESH);
  auto &g = static_cast<PxTriangleMeshGeometry const &>(g_);

  std::vector<float> vertices;
  vertices.reserve(3 * g.triangleMesh->getNbVertices());
  auto gv = g.triangleMesh->getVertices();
  for (uint32_t i = 0; i < g.triangleMesh->getNbVertices(); ++i) {
    vertices.push_back(gv[i].x);
    vertices.push_back(gv[i].y);
    vertices.push_back(gv[i].z);
  }
  return Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>>(
      vertices.data(), vertices.size() / 3, 3);
}

Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor>
PhysxCollisionShapeTriangleMesh::getTriangles() const {
  auto &g_ = mPxShape->getGeometry();
  assert(g_.getType() == PxGeometryType::eTRIANGLEMESH);
  auto &g = static_cast<PxTriangleMeshGeometry const &>(g_);

  std::vector<uint32_t> indices;
  indices.reserve(3 * g.triangleMesh->getNbTriangles());
  if (g.triangleMesh->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES) {
    auto gi = static_cast<const uint16_t *>(g.triangleMesh->getTriangles());
    for (uint32_t i = 0; i < g.triangleMesh->getNbTriangles(); ++i) {
      indices.push_back(gi[3 * i]);
      indices.push_back(gi[3 * i + 1]);
      indices.push_back(gi[3 * i + 2]);
    }
  } else {
    auto gi = static_cast<const uint32_t *>(g.triangleMesh->getTriangles());
    for (uint32_t i = 0; i < g.triangleMesh->getNbTriangles(); ++i) {
      indices.push_back(gi[3 * i]);
      indices.push_back(gi[3 * i + 1]);
      indices.push_back(gi[3 * i + 2]);
    }
  }

  return Eigen::Map<Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor>>(
      indices.data(), indices.size() / 3, 3);
}

// AABB
AABB PhysxCollisionShape::getGlobalAABBFast() const {
  if (!mParent) {
    throw std::runtime_error("failed to get global AABB: shape is not attached to a component");
  }
  return getTransformedAABB(getLocalAABB(), mParent->getPose() * getLocalPose());
}

AABB PhysxCollisionShape::computeGlobalAABBTight() const { return getGlobalAABBFast(); }

AABB PhysxCollisionShapePlane::getLocalAABB() const { return {{-1e7, -1e7, -1e7}, {0, 1e7, 1e7}}; }

AABB PhysxCollisionShapeBox::getLocalAABB() const {
  Vec3 h = getHalfLengths();
  return {-h, h};
}

AABB PhysxCollisionShapeCapsule::getLocalAABB() const {
  float r = getRadius();
  float h = getHalfLength();
  return {{-r - h, -r, -r}, {r + h, r, r}};
}

AABB PhysxCollisionShapeCylinder::getLocalAABB() const {
  float r = getRadius();
  float h = getHalfLength();
  return {{-h, -r, -r}, {h, r, r}};
}

AABB PhysxCollisionShapeSphere::getLocalAABB() const {
  float r = getRadius();
  return {{-r, -r, -r}, {r, r, r}};
}

AABB PhysxCollisionShapeConvexMesh::getLocalAABB() const { return mLocalAABB; }
AABB PhysxCollisionShapeConvexMesh::computeGlobalAABBTight() const {
  if (!mParent) {
    throw std::runtime_error("failed to get global AABB: shape is not attached to a component");
  }
  return computeAABB(getVertices(), getScale(), mParent->getPose() * getLocalPose());
}

AABB PhysxCollisionShapeTriangleMesh::getLocalAABB() const { return mLocalAABB; }
AABB PhysxCollisionShapeTriangleMesh::computeGlobalAABBTight() const {
  if (!mParent) {
    throw std::runtime_error("failed to get global AABB: shape is not attached to a component");
  }
  return computeAABB(getVertices(), getScale(), mParent->getPose() * getLocalPose());
}

} // namespace sapien::component
