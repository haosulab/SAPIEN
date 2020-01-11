#include "sapien_actor_base.h"
#include "renderer/render_interface.h"

namespace sapien {

std::vector<Renderer::IPxrRigidbody *> SActorBase::getRenderBodies() { return mRenderBodies; }

void SActorBase::updateRender(PxTransform const &pose) {
  for (auto body : mRenderBodies) {
    body->update(pose);
  }
}

SActorBase::SActorBase(physx_id_t id, SScene *scene,
                       std::vector<Renderer::IPxrRigidbody *> renderBodies)
    : mId(id), mParentScene(scene), mRenderBodies(renderBodies) {}

} // namespace sapien
