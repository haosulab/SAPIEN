#include "sapien_actor.h"
#include "renderer/render_interface.h"
#include "sapien_scene.h"

namespace sapien {

SActor::SActor(PxRigidBody *actor, physx_id_t id, SScene *scene,
               std::vector<Renderer::IPxrRigidbody *> renderBodies,
               std::vector<Renderer::IPxrRigidbody *> collisionBodies)
    : SActorDynamicBase(id, scene, renderBodies, collisionBodies), mActor(actor) {}

PxRigidBody *SActor::getPxActor() { return mActor; }

void SActor::destroy() { mParentScene->removeActor(this); }

EActorType SActor::getType() const {
  return mActor->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC) ? EActorType::KINEMATIC
                                                                        : EActorType::DYNAMIC;
}

void SActor::setPose(PxTransform const &pose) { getPxActor()->setGlobalPose(pose); }

SActorStatic::SActorStatic(PxRigidStatic *actor, physx_id_t id, SScene *scene,
                           std::vector<Renderer::IPxrRigidbody *> renderBodies,
                           std::vector<Renderer::IPxrRigidbody *> collisionBodies)
    : SActorBase(id, scene, renderBodies, collisionBodies), mActor(actor) {}

PxRigidActor *SActorStatic::getPxActor() { return mActor; }

void SActorStatic::destroy() { mParentScene->removeActor(this); }

void SActorStatic::setPose(PxTransform const &pose) { getPxActor()->setGlobalPose(pose); }

} // namespace sapien
