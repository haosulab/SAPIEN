#pragma once
#include "proto/render_server.grpc.pb.h"
#include "render_shape.h"
#include <sapien/component.h>

namespace sapien {
namespace render_server {

class ClientRenderBodyComponent : public Component {
public:
  ClientRenderBodyComponent();

  std::shared_ptr<ClientRenderBodyComponent> attachRenderShape(std::shared_ptr<ClientRenderShape>);
  std::vector<std::shared_ptr<ClientRenderShape>> getRenderShapes() const { return mRenderShapes; }

  void onAddToScene(Scene &scene) override;
  void onRemoveFromScene(Scene &scene) override;

private:
  std::vector<std::shared_ptr<ClientRenderShape>> mRenderShapes;
};

} // namespace render_server
} // namespace sapien
