#include "svulkan2_shape.h"
#include "svulkan2/resource/shape.h"
#include "svulkan2_material.h"
#include "svulkan2_renderer.h"

namespace sapien {
namespace Renderer {

std::shared_ptr<RenderMeshGeometry> SVulkan2RenderShape::getGeometry() const {
  auto position = mShape->mesh->getVertexAttribute("position");
  auto index = mShape->mesh->getIndices();
  auto mesh = std::make_shared<RenderMeshGeometry>();
  mesh->vertices = position;

  if (mShape->mesh->hasVertexAttribute("normal")) {
    mesh->normals = mShape->mesh->getVertexAttribute("normal");
  }
  if (mShape->mesh->hasVertexAttribute("tangent")) {
    mesh->tangents = mShape->mesh->getVertexAttribute("tangent");
  }
  if (mShape->mesh->hasVertexAttribute("bitangent")) {
    mesh->bitangents = mShape->mesh->getVertexAttribute("bitangent");
  }
  if (mShape->mesh->hasVertexAttribute("uv")) {
    mesh->bitangents = mShape->mesh->getVertexAttribute("uv");
  }
  mesh->indices = index;
  return mesh;
}

std::shared_ptr<IPxrMaterial> SVulkan2RenderShape::getMaterial() const {
  auto mat = std::dynamic_pointer_cast<svulkan2::resource::SVMetallicMaterial>(mShape->material);
  if (!mat) {
    throw std::runtime_error("invalid material");
  }
  return std::make_shared<SVulkan2Material>(mat, mParentBody->getScene()->getParentRenderer());
}

} // namespace Renderer
} // namespace sapien
