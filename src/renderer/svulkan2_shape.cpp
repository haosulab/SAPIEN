#include "svulkan2_shape.h"
#include "svulkan2/resource/shape.h"
#include "svulkan2_material.h"
#include "svulkan2_renderer.h"

namespace sapien {
namespace Renderer {

SVulkan2Geometry::SVulkan2Geometry(std::shared_ptr<svulkan2::resource::SVMesh> mesh)
    : mMesh(mesh) {}

std::vector<float> SVulkan2Geometry::getVertices() {
  return mMesh->getVertexAttribute("position");
}
std::vector<float> SVulkan2Geometry::getNormals() { return mMesh->getVertexAttribute("normal"); }
std::vector<float> SVulkan2Geometry::getUVs() { return mMesh->getVertexAttribute("uv"); }
std::vector<float> SVulkan2Geometry::getTangents() { return mMesh->getVertexAttribute("tangent"); }
std::vector<float> SVulkan2Geometry::getBitangents() {
  return mMesh->getVertexAttribute("bitangent");
}
std::vector<uint32_t> SVulkan2Geometry::getIndices() { return mMesh->getIndices(); }

void SVulkan2Geometry::setVertices(std::vector<float> const &vertices) {
  mMesh->setVertexAttribute("position", vertices, true);
}
void SVulkan2Geometry::setNormals(std::vector<float> const &normals) {
  mMesh->setVertexAttribute("normal", normals, true);
}
void SVulkan2Geometry::setUVs(std::vector<float> const &uvs) {
  mMesh->setVertexAttribute("uv", uvs, true);
}
void SVulkan2Geometry::setTangents(std::vector<float> const &tangents) {
  mMesh->setVertexAttribute("tangent", tangents, true);
}
void SVulkan2Geometry::setBitangents(std::vector<float> const &bitangents) {
  mMesh->setVertexAttribute("bitangents", bitangents, true);
}
void SVulkan2Geometry::setIndices(std::vector<uint32_t> const &indices) {
  mMesh->setIndices(indices);
}

std::shared_ptr<RenderMeshGeometry> SVulkan2RenderShape::getGeometry() const {
  return std::make_shared<SVulkan2Geometry>(mShape->mesh);
}

std::shared_ptr<IPxrMaterial> SVulkan2RenderShape::getMaterial() const {
  auto mat = std::dynamic_pointer_cast<svulkan2::resource::SVMetallicMaterial>(mShape->material);
  if (!mat) {
    throw std::runtime_error("invalid material");
  }
  return std::make_shared<SVulkan2Material>(mat, mParentBody->getScene()->getParentRenderer());
}

void SVulkan2RenderShape::setMaterial(std::shared_ptr<IPxrMaterial> material) {
  auto mat = std::dynamic_pointer_cast<SVulkan2Material>(material);
  if (!mat) {
    throw std::runtime_error("invalid material");
  }
  mShape->material = mat->getMaterial();
}

} // namespace Renderer
} // namespace sapien
