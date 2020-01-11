#include "sapien_actor.h"
#include "renderer/render_interface.h"
#include "sapien_scene.h"

namespace sapien {

SActor::SActor(PxRigidActor *actor, physx_id_t id, SScene *scene,
               std::vector<Renderer::IPxrRigidbody *> renderBodies)
    : SActorBase(id, scene, renderBodies), mActor(actor) {}

PxRigidActor *SActor::getPxActor() {
  return mActor;
}

void SActor::destroy() { mParentScene->removeActor(this); }

} // namespace sapien
