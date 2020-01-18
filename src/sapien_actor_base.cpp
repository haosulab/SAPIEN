#include "sapien_actor_base.h"
#include "renderer/render_interface.h"

namespace sapien {

std::vector<Renderer::IPxrRigidbody *> SActorBase::getRenderBodies() { return mRenderBodies; }
std::vector<Renderer::IPxrRigidbody *> SActorBase::getCollisionBodies() {
  return mCollisionBodies;
}

void SActorBase::setRenderMode(uint32_t mode) {
  if (mRenderMode != mode) {
    mRenderMode = mode;
    for (auto body : mRenderBodies) {
      body->setVisible(mode == 0);
    }
    for (auto body : mCollisionBodies) {
      body->setVisible(mode != 0);
    }
  }
}

uint32_t SActorBase::getRenderMode() { return mRenderMode; }

void SActorBase::updateRender(PxTransform const &pose) {
  if (mRenderMode == 0) {
    for (auto body : mRenderBodies) {
      body->update(pose);
    }
  } else {
    for (auto body : mCollisionBodies) {
      body->update(pose);
    }
  }
}

SActorBase::SActorBase(physx_id_t id, SScene *scene,
                       std::vector<Renderer::IPxrRigidbody *> renderBodies,
                       std::vector<Renderer::IPxrRigidbody *> collisionBodies)
    : mId(id), mParentScene(scene), mRenderBodies(renderBodies),
      mCollisionBodies(collisionBodies) {}

PxTransform SActorBase::getPose() { return getPxActor()->getGlobalPose(); }

PxVec3 SActorDynamicBase::getVel() { return getPxActor()->getLinearVelocity(); }

PxVec3 SActorDynamicBase::getAngularVel() { return getPxActor()->getAngularVelocity(); }

void SActorDynamicBase::addForceAtPoint(const PxVec3 &force, const PxVec3 &pos) {
  PxRigidBodyExt::addForceAtPos(*getPxActor(), force, pos);
}

} // namespace sapien
