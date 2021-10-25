#pragma once
#include "renderer/render_interface.h"
#include <svulkan2/core/context.h>
#include <svulkan2/renderer/renderer.h>
#include <svulkan2/scene/scene.h>
namespace sapien {
namespace Renderer {

class SVulkan2RenderShape : public IPxrRenderShape {
  std::shared_ptr<svulkan2::resource::SVShape> mShape;
  class SVulkan2Rigidbody *mParentBody;

public:
  inline SVulkan2RenderShape(std::shared_ptr<svulkan2::resource::SVShape> shape,
                             SVulkan2Rigidbody *body)
      : mShape(shape), mParentBody(body) {}
  [[nodiscard]] std::shared_ptr<RenderMeshGeometry> getGeometry() const override;
  [[nodiscard]] std::shared_ptr<IPxrMaterial> getMaterial() const override;
  void setMaterial(std::shared_ptr<IPxrMaterial> material) override;
};

} // namespace Renderer
} // namespace sapien
