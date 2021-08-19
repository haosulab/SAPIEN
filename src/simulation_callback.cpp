#include "simulation_callback.h"
#include "sapien_actor_base.h"
#include "sapien_contact.h"
#include "sapien_scene.h"
#include "sapien_trigger.h"
#include <iostream>

namespace sapien {

void DefaultEventCallback::onContact(const PxContactPairHeader &pairHeader,
                                     const PxContactPair *pairs, PxU32 nbPairs) {
  for (uint32_t i = 0; i < nbPairs; ++i) {
    std::unique_ptr<SContact> contact = std::make_unique<SContact>();
    contact->collisionShapes[0] = static_cast<SCollisionShape *>(pairs[i].shapes[0]->userData);
    contact->collisionShapes[1] = static_cast<SCollisionShape *>(pairs[i].shapes[1]->userData);
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
    EventActorContact event;
    event.self = contact->actors[0];
    event.other = contact->actors[1];
    event.contact = contact.get();
    contact->actors[0]->EventEmitter<EventActorContact>::emit(event);
    event.self = contact->actors[1];
    event.other = contact->actors[0];
    contact->actors[1]->EventEmitter<EventActorContact>::emit(event);
    mScene->updateContact(std::move(contact));
  }
}

void DefaultEventCallback::onAdvance(const PxRigidBody *const *bodyBuffer,
                                     const PxTransform *poseBuffer, const PxU32 count) {}
void DefaultEventCallback::onWake(PxActor **actors, PxU32 count) {}
void DefaultEventCallback::onSleep(PxActor **actors, PxU32 count) {}
void DefaultEventCallback::onConstraintBreak(PxConstraintInfo *constraints, PxU32 count) {}
void DefaultEventCallback::onTrigger(PxTriggerPair *pairs, PxU32 count) {
  for (PxU32 i = 0; i < count; i++) {
    // ignore pairs when shapes have been deleted
    if (pairs[i].flags &
        (PxTriggerPairFlag::eREMOVED_SHAPE_TRIGGER | PxTriggerPairFlag::eREMOVED_SHAPE_OTHER))
      continue;

    std::unique_ptr<STrigger> trigger = std::make_unique<STrigger>();

    trigger->triggerActor = static_cast<SActorBase *>(pairs[i].triggerActor->userData);
    trigger->otherActor = static_cast<SActorBase *>(pairs[i].otherActor->userData);
    trigger->starts = pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_FOUND;
    trigger->ends = pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_LOST;

    EventActorTrigger event;
    event.triggerActor = trigger->triggerActor;
    event.otherActor = trigger->otherActor;
    event.trigger = trigger.get();
    trigger->triggerActor->EventEmitter<EventActorTrigger>::emit(event);
  }
}

DefaultEventCallback::DefaultEventCallback(SScene *scene) : mScene(scene) {}

} // namespace sapien
