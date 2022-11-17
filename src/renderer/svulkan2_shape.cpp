#include "sapien/renderer/svulkan2_shape.h"
#include "sapien/renderer/dlpack.hpp"
#include <svulkan2/resource/shape.h>
#include "sapien/renderer/svulkan2_material.h"
#include "sapien/renderer/svulkan2_renderer.h"
#include "sapien/renderer/svulkan2_rigidbody.h"

namespace sapien {
namespace Renderer {

SVulkan2Mesh::SVulkan2Mesh(std::shared_ptr<svulkan2::resource::SVMesh> mesh) : mMesh(mesh) {}

std::vector<float> SVulkan2Mesh::getVertices() { return mMesh->getVertexAttribute("position"); }
std::vector<float> SVulkan2Mesh::getNormals() { return mMesh->getVertexAttribute("normal"); }
std::vector<float> SVulkan2Mesh::getUVs() { return mMesh->getVertexAttribute("uv"); }
std::vector<float> SVulkan2Mesh::getTangents() { return mMesh->getVertexAttribute("tangent"); }
std::vector<float> SVulkan2Mesh::getBitangents() { return mMesh->getVertexAttribute("bitangent"); }
std::vector<uint32_t> SVulkan2Mesh::getIndices() { return mMesh->getIndices(); }

void SVulkan2Mesh::setVertices(std::vector<float> const &vertices) {
  mMesh->setVertexAttribute("position", vertices, true);
}
void SVulkan2Mesh::setNormals(std::vector<float> const &normals) {
  mMesh->setVertexAttribute("normal", normals, true);
}
void SVulkan2Mesh::setUVs(std::vector<float> const &uvs) {
  mMesh->setVertexAttribute("uv", uvs, true);
}
void SVulkan2Mesh::setTangents(std::vector<float> const &tangents) {
  mMesh->setVertexAttribute("tangent", tangents, true);
}
void SVulkan2Mesh::setBitangents(std::vector<float> const &bitangents) {
  mMesh->setVertexAttribute("bitangents", bitangents, true);
}
void SVulkan2Mesh::setIndices(std::vector<uint32_t> const &indices) { mMesh->setIndices(indices); }

#ifdef SAPIEN_DLPACK
DLManagedTensor *SVulkan2Mesh::getDLVertices() {
  auto &buffer = mMesh->getVertexBuffer();
  void *ptr = buffer.getCudaPtr();
  int id = buffer.getCudaDeviceId();

  int64_t vertexCount = mMesh->getVertexCount();
  int64_t vertexSize = mMesh->getVertexSize();

  return dl_wrapper<svulkan2::resource::SVMesh>(mMesh, ptr, id, {vertexCount, vertexSize / 4},
                                                {DLDataTypeCode::kDLFloat, 32, 1});
}
#endif

std::shared_ptr<IRenderMesh> SVulkan2RenderShape::getGeometry() const {
  return std::make_shared<SVulkan2Mesh>(mShape->mesh);
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
