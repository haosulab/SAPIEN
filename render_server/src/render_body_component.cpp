#include "render_body_component.h"
#include "client_system.h"
#include <sapien/scene.h>

namespace sapien {
namespace render_server {

ClientRenderBodyComponent::ClientRenderBodyComponent() {}

std::shared_ptr<ClientRenderBodyComponent>
ClientRenderBodyComponent::attachRenderShape(std::shared_ptr<ClientRenderShape> shape) {
  if (getScene()) {
    throw std::runtime_error("failed to add shape: component already added to scene");
  }
  mRenderShapes.push_back(shape);
  return std::static_pointer_cast<ClientRenderBodyComponent>(shared_from_this());
}

void ClientRenderBodyComponent::onAddToScene(Scene &scene) {
  auto system = scene.getSystemWithType<ClientSystem>("render_server");
  for (auto shape : mRenderShapes) {
    if (auto s = std::dynamic_pointer_cast<ClientRenderShapeTriangleMesh>(shape)) {
      auto renderId = system->nextRenderId();
      s->internalSetRenderId(renderId);
      uint64_t entityId = getEntity()->getPerSceneId();

      grpc::ClientContext context;
      proto::AddBodyMeshReq req;
      proto::Id res;
      req.set_scene_id(mId);
      req.set_filename(s->getFilename());
      req.mutable_scale()->set_x(s->getScale().x);
      req.mutable_scale()->set_y(s->getScale().y);
      req.mutable_scale()->set_z(s->getScale().z);
      req.set_segmentation0(renderId);
      req.set_segmentation1(entityId);

      auto status = system->getStub().AddBodyMesh(&context, req, &res);
      if (!status.ok()) {
        throw std::runtime_error(status.error_message());
      }

      s->internalSetServerId(res.id());
    }
  }
}

void ClientRenderBodyComponent::onRemoveFromScene(Scene &scene) {
  // TODO
}

} // namespace render_server
} // namespace sapien
