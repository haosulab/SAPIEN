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

PxTransform SActorBase::getPose() { return getPxActor()->getGlobalPose(); }

PxVec3 SActorDynamicBase::getVel() {
  return getPxRigidBody()->getLinearVelocity();
}

PxVec3 SActorDynamicBase::getAngularVel() {
  return getPxRigidBody()->getAngularVelocity();
}

PxRigidActor* SActorDynamicBase::getPxActor() {
  return getPxRigidBody();
}

void SActorDynamicBase::addForceAtPoint(const PxVec3 &force, const PxVec3 &pos) {
  PxRigidBodyExt::addForceAtPos(*getPxRigidBody(), force, pos);
}

} // namespace sapien
