#pragma once
#include "render_interface.h"
#include <svulkan2/core/context.h>
#include <svulkan2/renderer/renderer.h>
#include <svulkan2/scene/scene.h>
namespace sapien {
namespace Renderer {

class SVulkan2Mesh : public IRenderMesh {
  std::shared_ptr<svulkan2::resource::SVMesh> mMesh;

public:
  SVulkan2Mesh(std::shared_ptr<svulkan2::resource::SVMesh> mesh);

  std::vector<float> getVertices() override;
  std::vector<float> getNormals() override;
  std::vector<float> getUVs() override;
  std::vector<float> getTangents() override;
  std::vector<float> getBitangents() override;
  std::vector<uint32_t> getIndices() override;
  void setVertices(std::vector<float> const &) override;
  void setNormals(std::vector<float> const &) override;
  void setUVs(std::vector<float> const &) override;
  void setTangents(std::vector<float> const &) override;
  void setBitangents(std::vector<float> const &) override;
  void setIndices(std::vector<uint32_t> const &) override;

#ifdef SAPIEN_DLPACK
  DLManagedTensor *getDLVertices();
#endif


  inline std::shared_ptr<svulkan2::resource::SVMesh> getMesh() const { return mMesh; }
};

class SVulkan2RenderShape : public IPxrRenderShape {
  std::shared_ptr<svulkan2::resource::SVShape> mShape;
  class SVulkan2Rigidbody *mParentBody;

public:
  inline SVulkan2RenderShape(std::shared_ptr<svulkan2::resource::SVShape> shape,
                             SVulkan2Rigidbody *body)
      : mShape(shape), mParentBody(body) {}
  [[nodiscard]] std::shared_ptr<IRenderMesh> getGeometry() const override;
  [[nodiscard]] std::shared_ptr<IPxrMaterial> getMaterial() const override;
  void setMaterial(std::shared_ptr<IPxrMaterial> material) override;
};

} // namespace Renderer
} // namespace sapien
