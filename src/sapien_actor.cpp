#include "sapien_actor.h"
#include "renderer/render_interface.h"
#include "sapien_scene.h"
#include <spdlog/spdlog.h>

namespace sapien {

SActor::SActor(PxRigidDynamic *actor, physx_id_t id, SScene *scene,
               std::vector<Renderer::IPxrRigidbody *> renderBodies,
               std::vector<Renderer::IPxrRigidbody *> collisionBodies)
    : SActorDynamicBase(id, scene, renderBodies, collisionBodies), mActor(actor) {
  actor->userData = this;
}

PxRigidDynamic *SActor::getPxActor() { return mActor; }

void SActor::destroy() { mParentScene->removeActor(this); }

EActorType SActor::getType() const {
  return mActor->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC) ? EActorType::KINEMATIC
                                                                        : EActorType::DYNAMIC;
}

void SActor::setPose(PxTransform const &pose) { getPxActor()->setGlobalPose(pose); }

void SActor::setVelocity(PxVec3 const &v) { getPxActor()->setLinearVelocity(v); }
void SActor::setAngularVelocity(PxVec3 const &v) { getPxActor()->setAngularVelocity(v); }
void SActor::lockMotion(bool x, bool y, bool z, bool ax, bool ay, bool az) {
  auto flags = PxRigidDynamicLockFlags();
  if (x) {
    flags |= PxRigidDynamicLockFlag::eLOCK_LINEAR_X;
  }
  if (y) {
    flags |= PxRigidDynamicLockFlag::eLOCK_LINEAR_Y;
  }
  if (z) {
    flags |= PxRigidDynamicLockFlag::eLOCK_LINEAR_Z;
  }
  if (ax) {
    flags |= PxRigidDynamicLockFlag::eLOCK_ANGULAR_X;
  }
  if (ay) {
    flags |= PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y;
  }
  if (az) {
    flags |= PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z;
  }
  mActor->setRigidDynamicLockFlags(flags);
}

void SActor::setSolverIterations(uint32_t position, uint32_t velocity) {
  getPxActor()->setSolverIterationCounts(position, velocity);
}

void SActor::prestep() {
  SActorDynamicBase::prestep();
  if (mActor->getAngularVelocity().magnitudeSquared() >= 1e-4) {
    PxVec3 I = mActor->getMassSpaceInertiaTensor();
    PxVec3 w = mActor->getAngularVelocity();
    PxQuat orient = mActor->getGlobalPose().q * mActor->getCMassLocalPose().q;
    PxVec3 C = orient.rotate(orient.rotateInv(w).multiply(I)).cross(w);
    mActor->addTorque(C, PxForceMode::eFORCE, false);
  }
}

std::vector<PxReal> SActor::packData() {
  std::vector<PxReal> data;
  auto pose = getPose();
  data.push_back(pose.p.x);
  data.push_back(pose.p.y);
  data.push_back(pose.p.z);
  data.push_back(pose.q.x);
  data.push_back(pose.q.y);
  data.push_back(pose.q.z);
  data.push_back(pose.q.w);

  if (getType() == EActorType::DYNAMIC) {
    auto lv = getVelocity();
    auto av = getAngularVelocity();
    data.push_back(lv.x);
    data.push_back(lv.y);
    data.push_back(lv.z);
    data.push_back(av.x);
    data.push_back(av.y);
    data.push_back(av.z);
  }

  return data;
}

void SActor::unpackData(std::vector<PxReal> const &data) {
  if (getType() == EActorType::DYNAMIC) {
    if (data.size() != 13) {
      spdlog::get("SAPIEN")->error("Failed to unpack actor: {} numbers expected but {} provided",
                                   13, data.size());
      return;
    }
    getPxActor()->setGlobalPose(
        {{data[0], data[1], data[2]}, {data[3], data[4], data[5], data[6]}});
    getPxActor()->setLinearVelocity({data[7], data[8], data[9]});
    getPxActor()->setAngularVelocity({data[10], data[11], data[12]});
  } else {
    if (data.size() != 7) {
      spdlog::get("SAPIEN")->error("Failed to unpack actor: {} numbers expected but {} provided",
                                   7, data.size());
      return;
    }
    getPxActor()->setGlobalPose(
        {{data[0], data[1], data[2]}, {data[3], data[4], data[5], data[6]}});
  }
}

SActorStatic::SActorStatic(PxRigidStatic *actor, physx_id_t id, SScene *scene,
                           std::vector<Renderer::IPxrRigidbody *> renderBodies,
                           std::vector<Renderer::IPxrRigidbody *> collisionBodies)
    : SActorBase(id, scene, renderBodies, collisionBodies), mActor(actor) {
  actor->userData = this;
}

PxRigidActor *SActorStatic::getPxActor() { return mActor; }

void SActorStatic::destroy() { mParentScene->removeActor(this); }

void SActorStatic::setPose(PxTransform const &pose) { getPxActor()->setGlobalPose(pose); }

std::vector<PxReal> SActorStatic::packData() {
  std::vector<PxReal> data;
  auto pose = getPose();
  data.push_back(pose.p.x);
  data.push_back(pose.p.y);
  data.push_back(pose.p.z);
  data.push_back(pose.q.x);
  data.push_back(pose.q.y);
  data.push_back(pose.q.z);
  data.push_back(pose.q.w);

  return data;
}

void SActorStatic::unpackData(std::vector<PxReal> const &data) {
  if (data.size() != 7) {
    spdlog::get("SAPIEN")->error("Failed to unpack actor: {} numbers expected but {} provided", 13,
                                 data.size());
    return;
  }
  getPxActor()->setGlobalPose({{data[0], data[1], data[2]}, {data[3], data[4], data[5], data[6]}});
}

} // namespace sapien
