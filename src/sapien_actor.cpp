#include "sapien_actor.h"
#include "renderer/render_interface.h"
#include "sapien_scene.h"

namespace sapien {

SActor::SActor(PxRigidActor *actor, physx_id_t id, SScene *scene,
               std::vector<Renderer::IPxrRigidbody *> renderBodies)
    : mActor(actor), mId(id), mParentScene(scene), mRenderBodies(renderBodies) {}

void SActor::updateRender(PxTransform const &pose) {
  for (auto body : mRenderBodies) {
    body->update(pose);
  }
}

std::vector<Renderer::IPxrRigidbody *> SActor::getRenderBodies() {
  return mRenderBodies;
}

void SActor::destroy() { mParentScene->removeActor(this); }

} // namespace sapien
