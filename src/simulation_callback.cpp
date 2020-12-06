#include "simulation_callback.h"
#include "sapien_actor_base.h"
#include "sapien_contact.h"
#include "sapien_scene.h"
#include <iostream>

namespace sapien {

void DefaultEventCallback::onContact(const PxContactPairHeader &pairHeader,
                                     const PxContactPair *pairs, PxU32 nbPairs) {
  // PxContactPairExtraDataIterator iter(pairHeader.extraDataStream,
  //                                     pairHeader.extraDataStreamSize);
  // while(iter.nextItemSet())
  // {
  //   if (iter.postSolverVelocity)
  //   {
  //     PxVec3 linearVelocityActor0 = iter.postSolverVelocity->linearVelocity[0];
  //     PxVec3 linearVelocityActor1 = iter.postSolverVelocity->linearVelocity[1];
  //   }
  // }

  for (uint32_t i = 0; i < nbPairs; ++i) {
    std::unique_ptr<SContact> contact = std::make_unique<SContact>();
    contact->actors[0] = static_cast<SActorBase *>(pairHeader.actors[0]->userData);
    contact->actors[1] = static_cast<SActorBase *>(pairHeader.actors[1]->userData);

    contact->starts = pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_FOUND;
    contact->ends = pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_LOST;
    contact->persists = pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_PERSISTS;

    std::vector<PxContactPairPoint> points(pairs[i].contactCount);
    pairs[i].extractContacts(points.data(), pairs[i].contactCount);

    for (auto &p : points) {
      contact->points.push_back({p.position, p.normal, p.impulse, p.separation});
    }
    contact->actors[0]->notifyContact(*contact->actors[1], *contact);
    contact->actors[1]->notifyContact(*contact->actors[0], *contact);
    mScene->updateContact(pairs[i].shapes[0], pairs[i].shapes[1], std::move(contact));
  }
}

void DefaultEventCallback::onAdvance(const PxRigidBody *const *bodyBuffer,
                                     const PxTransform *poseBuffer, const PxU32 count) {}
void DefaultEventCallback::onWake(PxActor **actors, PxU32 count) {}
void DefaultEventCallback::onSleep(PxActor **actors, PxU32 count) {}
void DefaultEventCallback::onConstraintBreak(PxConstraintInfo *constraints, PxU32 count) {}
void DefaultEventCallback::onTrigger(PxTriggerPair *pairs, PxU32 count) {}

DefaultEventCallback::DefaultEventCallback(SScene *scene) : mScene(scene) {}

} // namespace sapien
