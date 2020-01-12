#include "simulation_callback.h"
#include "sapien_contact.h"
#include "sapien_scene.h"

namespace sapien {

void DefaultEventCallback::onContact(const PxContactPairHeader &pairHeader,
                                     const PxContactPair *pairs, PxU32 nbPairs) {
  SContact contact;
  contact.actors[0] = static_cast<SActorBase *>(pairHeader.actors[0]->userData);
  contact.actors[1] = static_cast<SActorBase *>(pairHeader.actors[1]->userData);
  for (uint32_t i = 0; i < nbPairs; ++i) {
    std::vector<PxContactPairPoint> points(pairs[i].contactCount);
    pairs[i].extractContacts(points.data(), pairs[i].contactCount);
    for (auto &p : points) {
      contact.point = p.position;
      contact.normal = p.normal;
      contact.impulse = p.impulse;
      contact.separation = p.separation;
      mScene->addContact(contact);
    }
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
