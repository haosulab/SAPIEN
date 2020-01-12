#include "sapien_actor.h"
#include "renderer/render_interface.h"
#include "sapien_scene.h"

namespace sapien {

SActor::SActor(PxRigidBody *actor, physx_id_t id, SScene *scene,
               std::vector<Renderer::IPxrRigidbody *> renderBodies)
    : SActorDynamicBase(id, scene, renderBodies), mActor(actor) {}

PxRigidBody *SActor::getPxRigidBody() { return mActor; }

void SActor::destroy() { mParentScene->removeActor(this); }

void SActor::setPose(PxTransform const &pose) { getPxActor()->setGlobalPose(pose); }

SActorStatic::SActorStatic(PxRigidStatic *actor, physx_id_t id, SScene *scene,
                           std::vector<Renderer::IPxrRigidbody *> renderBodies)
    : SActorBase(id, scene, renderBodies), mActor(actor) {}

PxRigidActor *SActorStatic::getPxActor() { return mActor; }

void SActorStatic::destroy() { mParentScene->removeActor(this); }

void SActorStatic::setPose(PxTransform const &pose) { getPxActor()->setGlobalPose(pose); }

} // namespace sapien
