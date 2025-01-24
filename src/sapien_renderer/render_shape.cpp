/*
 * Copyright 2025 Hillbot Inc.
 * Copyright 2020-2024 UCSD SU Lab
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "sapien/sapien_renderer/render_shape.h"
#include "sapien/sapien_renderer/render_body_component.h"

namespace sapien {
namespace sapien_renderer {

std::shared_ptr<SapienRenderBodyComponent> RenderShape::getParent() const {
  if (!mParent) {
    return nullptr;
  }
  return std::static_pointer_cast<SapienRenderBodyComponent>(mParent->shared_from_this());
}

RenderShape::RenderShape() { mEngine = SapienRenderEngine::Get(); }

void RenderShape::setLocalPose(Pose const &pose) {
  if (mParent) {
    throw std::runtime_error(
        "failed to set local pose: local pose cannot be modified once attached to component");
  }
  mLocalPose = pose;
}

vk::FrontFace RenderShape::getFrontFace() const { return mFrontFace; }
void RenderShape::setFrontFace(vk::FrontFace frontFace) {
  if (mParent) {
    throw std::runtime_error(
        "failed to set front face: it cannot be modified once attached to component");
  }
  mFrontFace = frontFace;
}

Pose RenderShape::getLocalPose() const { return mLocalPose; }
RenderShape::~RenderShape() {}

svulkan2::scene::Transform RenderShape::getLocalTransform() const {
  svulkan2::scene::Transform t;
  t.position = {mLocalPose.p.x, mLocalPose.p.y, mLocalPose.p.z};
  t.rotation = {mLocalPose.q.w, mLocalPose.q.x, mLocalPose.q.y, mLocalPose.q.z};
  t.scale = {mScale.x, mScale.y, mScale.z};
  return t;
}

int RenderShape::getInternalGpuTransformIndex() {
  if (!mObject) {
    throw std::runtime_error("the shape is not added to scene");
  }
  mObject->getScene()->prepareObjectTransformBuffer();
  return mObject->getInternalGpuIndex();
}

Vec3 RenderShape::getGpuScale() { return mScale; }

Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>
RenderShapePrimitive::getVertices() const {
  auto mesh = getModel()->getShapes().at(0)->mesh;
  auto position = mesh->getVertexAttribute("position");
  return Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>>(
      position.data(), position.size() / 3, 3);
}

Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor>
RenderShapePrimitive::getTriangles() const {
  auto mesh = getModel()->getShapes().at(0)->mesh;
  auto indices = mesh->getIndices();
  return Eigen::Map<Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor>>(
      indices.data(), indices.size() / 3, 3);
}

Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> RenderShapePrimitive::getNormal() const {
  auto mesh = getModel()->getShapes().at(0)->mesh;
  auto normal = mesh->getVertexAttribute("normal");
  return Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>>(
      normal.data(), normal.size() / 3, 3);
}

Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> RenderShapePrimitive::getUV() const {
  auto mesh = getModel()->getShapes().at(0)->mesh;
  auto uv = mesh->getVertexAttribute("uv");
  return Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor>>(uv.data(),
                                                                              uv.size() / 2, 2);
}

RenderShapePlane::RenderShapePlane(Vec3 scale, std::shared_ptr<SapienRenderMaterial> material) {
  auto mesh = mEngine->getPlaneMesh();
  mMaterial = material;
  mScale = scale;
  auto shape = svulkan2::resource::SVShape::Create(mesh, material->getMaterial());
  mModel = svulkan2::resource::SVModel::FromData({shape});
}
Vec3 RenderShapePlane::getScale() const { return mScale; }

RenderShapeBox::RenderShapeBox(Vec3 halfLengths, std::shared_ptr<SapienRenderMaterial> material) {
  auto mesh = mEngine->getBoxMesh();
  mMaterial = material;
  mScale = halfLengths;
  auto shape = svulkan2::resource::SVShape::Create(mesh, material->getMaterial());
  mModel = svulkan2::resource::SVModel::FromData({shape});
}
Vec3 RenderShapeBox::getHalfLengths() const { return mScale; }

RenderShapeCapsule::RenderShapeCapsule(float radius, float halfLength,
                                       std::shared_ptr<SapienRenderMaterial> material)
    : mRadius(radius), mHalfLength(halfLength) {
  auto mesh = svulkan2::resource::SVMesh::CreateCapsule(radius, halfLength, 32, 8);
  mMaterial = material;
  mScale = {1.f, 1.f, 1.f};
  auto shape = svulkan2::resource::SVShape::Create(mesh, material->getMaterial());
  mModel = svulkan2::resource::SVModel::FromData({shape});
}
float RenderShapeCapsule::getRadius() const { return mRadius; }
float RenderShapeCapsule::getHalfLength() const { return mHalfLength; }

RenderShapeCylinder::RenderShapeCylinder(float radius, float halfLength,
                                         std::shared_ptr<SapienRenderMaterial> material)
    : mRadius(radius), mHalfLength(halfLength) {
  auto mesh = svulkan2::resource::SVMesh::CreateCylinder(32);
  auto shape = svulkan2::resource::SVShape::Create(mesh, material->getMaterial());
  mMaterial = material;
  mScale = {halfLength, radius, radius};
  mModel = svulkan2::resource::SVModel::FromData({shape});
}
float RenderShapeCylinder::getRadius() const { return mRadius; }
float RenderShapeCylinder::getHalfLength() const { return mHalfLength; }

RenderShapeSphere::RenderShapeSphere(float radius,
                                     std::shared_ptr<SapienRenderMaterial> material) {
  auto mesh = mEngine->getSphereMesh();
  mMaterial = material;
  mScale = {radius, radius, radius};
  auto shape = svulkan2::resource::SVShape::Create(mesh, material->getMaterial());
  mModel = svulkan2::resource::SVModel::FromData({shape});
}
float RenderShapeSphere::getRadius() const { return mScale.x; }

RenderShapeTriangleMeshPart::RenderShapeTriangleMeshPart(
    std::shared_ptr<svulkan2::resource::SVShape> shape)
    : mShape(shape) {
  if (!shape || !shape->material || !shape->mesh) {
    throw std::runtime_error("failed to create mesh part: invalid model");
  }
}

Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>
RenderShapeTriangleMeshPart::getVertices() const {
  auto position = mShape->mesh->getVertexAttribute("position");
  return Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>>(
      position.data(), position.size() / 3, 3);
}

CudaArrayHandle RenderShapeTriangleMeshPart::getVertexBufferCudaArray() const {
#ifdef SAPIEN_CUDA
  int count = mShape->mesh->getVertexCount();
  int channels = mShape->mesh->getVertexSize() / 4;
  int itemsize = 4;

  assert(static_cast<int>(mShape->mesh->getVertexBuffer().getSize()) ==
         count * channels * itemsize);

  return CudaArrayHandle{.shape = {count, channels},
                         .strides = {channels * itemsize, itemsize},
                         .type = "f4",
                         .cudaId = mShape->mesh->getVertexBuffer().getCudaDeviceId(),
                         .ptr = mShape->mesh->getVertexBuffer().getCudaPtr()};
#else
  throw std::runtime_error("sapien is not copmiled with CUDA support");
#endif
}

CudaArrayHandle RenderShapeTriangleMeshPart::getIndexBufferCudaArray() const {
#ifdef SAPIEN_CUDA
  int count = mShape->mesh->getTriangleCount();
  int channels = 3;
  int itemsize = 4;

  assert(static_cast<int>(mShape->mesh->getIndexBuffer().getSize()) ==
         count * channels * itemsize);

  return CudaArrayHandle{.shape = {count, channels},
                         .strides = {channels * itemsize, itemsize},
                         .type = "u4",
                         .cudaId = mShape->mesh->getIndexBuffer().getCudaDeviceId(),
                         .ptr = mShape->mesh->getIndexBuffer().getCudaPtr()};
#else
  throw std::runtime_error("sapien is not copmiled with CUDA support");
#endif
}

Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor>
RenderShapeTriangleMeshPart::getTriangles() const {
  auto indices = mShape->mesh->getIndices();
  return Eigen::Map<Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor>>(
      indices.data(), indices.size() / 3, 3);
}

void RenderShapeTriangleMeshPart::setUV(
    Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> const &uv) {
  mShape->mesh->setVertexAttribute("uv", std::vector<float>(uv.data(), uv.data() + uv.size()));
}

void RenderShapeTriangleMeshPart::setNormal(
    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &normal) {
  mShape->mesh->setVertexAttribute(
      "normal", std::vector<float>(normal.data(), normal.data() + normal.size()));
}

Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor>
RenderShapeTriangleMeshPart::getUV() const {
  auto uv = mShape->mesh->getVertexAttribute("uv");
  return Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor>>(uv.data(),
                                                                              uv.size() / 2, 2);
}
Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>
RenderShapeTriangleMeshPart::getNormal() const {
  auto normal = mShape->mesh->getVertexAttribute("normal");
  return Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>>(
      normal.data(), normal.size() / 3, 3);
}

std::shared_ptr<SapienRenderMaterial> RenderShapeTriangleMeshPart::getMaterial() const {
  return std::make_shared<SapienRenderMaterial>(
      std::dynamic_pointer_cast<svulkan2::resource::SVMetallicMaterial>(mShape->material));
}

RenderShapeTriangleMesh::RenderShapeTriangleMesh(std::string const &filename, Vec3 scale,
                                                 std::shared_ptr<SapienRenderMaterial> material)
    : mFilename(filename) {
  mModel = mEngine->getContext()->getResourceManager()->CreateModelFromFile(filename);
  mMaterial = material;
  mScale = scale;
  if (material) {
    mModel->loadAsync().get();
    std::vector<std::shared_ptr<svulkan2::resource::SVShape>> shapes;
    for (auto s : mModel->getShapes()) {
      auto shape = svulkan2::resource::SVShape::Create(s->mesh, material->getMaterial());
      shapes.push_back(shape);
    }
    mModel = svulkan2::resource::SVModel::FromData(shapes);
  }
}

RenderShapeTriangleMesh::RenderShapeTriangleMesh(
    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &vertices,
    Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor> const &triangles,
    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &normals,
    Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> const &uvs,
    std::shared_ptr<SapienRenderMaterial> material) {

  std::vector<float> vs(vertices.data(), vertices.data() + vertices.size());
  std::vector<uint32_t> ts(triangles.data(), triangles.data() + triangles.size());
  std::vector<float> ns(normals.data(), normals.data() + normals.size());
  std::vector<float> uvs_(uvs.data(), uvs.data() + uvs.size());

  auto mesh = svulkan2::resource::SVMesh::Create(vs, ts);
  mMaterial = material;
  mScale = {1.f, 1.f, 1.f};
  if (ns.size()) {
    mesh->setVertexAttribute("normal", ns);
  }
  if (uvs_.size()) {
    mesh->setVertexAttribute("uv", uvs_);
  }
  auto shape = svulkan2::resource::SVShape::Create(mesh, material->getMaterial());
  mModel = svulkan2::resource::SVModel::FromData({shape});
}

RenderShapeTriangleMesh::RenderShapeTriangleMesh(
    std::vector<std::shared_ptr<RenderShapeTriangleMeshPart>> parts) {
  std::vector<std::shared_ptr<svulkan2::resource::SVShape>> shapes;
  for (auto part : parts) {
    shapes.push_back(part->getShape());
  }
  mModel = svulkan2::resource::SVModel::FromData(shapes);
}

std::string RenderShapeTriangleMesh::getFilename() const { return mFilename; }
Vec3 RenderShapeTriangleMesh::getScale() const { return mScale; }

void RenderShapeTriangleMesh::setScale(Vec3 const &scale) {
  if (mParent) {
    throw std::runtime_error("failed to set scale: only allowed when not attached to component");
  }
  mScale = scale;
}

std::vector<std::shared_ptr<RenderShapeTriangleMeshPart>> RenderShapeTriangleMesh::getParts() {
  std::vector<std::shared_ptr<RenderShapeTriangleMeshPart>> parts;
  for (auto shape : mModel->getShapes()) {
    parts.push_back({std::make_shared<RenderShapeTriangleMeshPart>(shape)});
  }
  return parts;
}

std::shared_ptr<SapienRenderMaterial> RenderShapeTriangleMesh::getMaterial() const {
  if (mMaterial) {
    return mMaterial;
  }

  std::shared_ptr<SapienRenderMaterial> material{};
  for (auto &s : mModel->getShapes()) {
    if (!material) {
      material = std::make_shared<SapienRenderMaterial>(
          std::dynamic_pointer_cast<svulkan2::resource::SVMetallicMaterial>(s->material));
    } else if (s->material != material->getMaterial()) {
      throw std::runtime_error("Triangle shape contains multiple parts with different materials.");
    }
  }
  return material;
}

// AABB
AABB RenderShape::getGlobalAABBFast() {
  if (!mParent) {
    throw std::runtime_error("failed to get global AABB: shape is not attached to a component");
  }
  return getTransformedAABB(getLocalAABB(), mParent->getPose() * getLocalPose());
}
AABB RenderShape::computeGlobalAABBTight() { return getGlobalAABBFast(); }

void RenderShape::setGpuBatchedPoseIndex(int index) { mBatchedPoseIndex = index; }
int RenderShape::getGpuBatchedPoseIndex() const { return mBatchedPoseIndex; }

AABB RenderShapePlane::getLocalAABB() {
  Vec3 h = getScale();
  return {-h, h};
}
AABB RenderShapeBox::getLocalAABB() {
  Vec3 h = getHalfLengths();
  return {-h, h};
}
AABB RenderShapeSphere::getLocalAABB() {
  float r = getRadius();
  return {{-r, -r, -r}, {r, r, r}};
}
AABB RenderShapeCapsule::getLocalAABB() {
  float r = getRadius();
  float h = getHalfLength();
  return {{-r - h, -r, -r}, {r + h, r, r}};
}
AABB RenderShapeCylinder::getLocalAABB() {
  float r = getRadius();
  float h = getHalfLength();
  return {{-h, -r, -r}, {h, r, r}};
}
AABB RenderShapeTriangleMesh::getLocalAABB() {
  if (mAABB.has_value()) {
    return mAABB.value();
  }

  auto parts = getParts();
  if (parts.size() == 0) {
    throw std::runtime_error("failed to get global AABB: triangle mesh is empty");
  }
  AABB aabb = computeAABB(parts[0]->getVertices(), getScale(), Pose());
  for (uint32_t i = 1; i < parts.size(); ++i) {
    aabb = aabb + computeAABB(parts[i]->getVertices(), getScale(), Pose());
  }
  mAABB = aabb;
  return aabb;
}

AABB RenderShapeTriangleMesh::computeGlobalAABBTight() {
  if (!mParent) {
    throw std::runtime_error("failed to get global AABB: shape is not attached to a component");
  }

  auto parts = getParts();
  if (parts.size() == 0) {
    throw std::runtime_error("failed to get global AABB: triangle mesh is empty");
  }
  AABB aabb =
      computeAABB(parts[0]->getVertices(), getScale(), mParent->getPose() * getLocalPose());
  for (uint32_t i = 1; i < parts.size(); ++i) {
    aabb = aabb +
           computeAABB(parts[i]->getVertices(), getScale(), mParent->getPose() * getLocalPose());
  }
  return aabb;
}

std::shared_ptr<RenderShape> RenderShapePlane::clone() const {
  auto newShape = std::make_shared<RenderShapePlane>(getScale(), getMaterial());
  newShape->setFrontFace(getFrontFace());
  newShape->setName(getName());
  newShape->setLocalPose(getLocalPose());
  return newShape;
}

std::shared_ptr<RenderShape> RenderShapeBox::clone() const {
  auto newShape = std::make_shared<RenderShapeBox>(getHalfLengths(), getMaterial());
  newShape->setFrontFace(getFrontFace());
  newShape->setName(getName());
  newShape->setLocalPose(getLocalPose());
  return newShape;
}

std::shared_ptr<RenderShape> RenderShapeSphere::clone() const {
  auto newShape = std::make_shared<RenderShapeSphere>(getRadius(), getMaterial());
  newShape->setFrontFace(getFrontFace());
  newShape->setName(getName());
  newShape->setLocalPose(getLocalPose());
  return newShape;
}

std::shared_ptr<RenderShape> RenderShapeCapsule::clone() const {
  auto newShape =
      std::make_shared<RenderShapeCapsule>(getRadius(), getHalfLength(), getMaterial());
  newShape->setFrontFace(getFrontFace());
  newShape->setName(getName());
  newShape->setLocalPose(getLocalPose());
  return newShape;
}

std::shared_ptr<RenderShape> RenderShapeCylinder::clone() const {
  auto newShape =
      std::make_shared<RenderShapeCylinder>(getRadius(), getHalfLength(), getMaterial());
  newShape->setFrontFace(getFrontFace());
  newShape->setName(getName());
  newShape->setLocalPose(getLocalPose());
  return newShape;
}

std::shared_ptr<RenderShape> RenderShapeTriangleMesh::clone() const {
  std::vector<std::shared_ptr<RenderShapeTriangleMeshPart>> parts;
  for (auto shape : mModel->getShapes()) {
    parts.push_back(std::make_shared<RenderShapeTriangleMeshPart>(shape));
  }
  auto newShape = std::make_shared<RenderShapeTriangleMesh>(parts);
  newShape->mScale = mScale;
  newShape->mFilename = mFilename;
  newShape->mMaterial = mMaterial;

  newShape->setFrontFace(getFrontFace());
  newShape->setName(getName());
  newShape->setLocalPose(getLocalPose());
  return newShape;
}

} // namespace sapien_renderer
} // namespace sapien
