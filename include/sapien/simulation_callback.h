#pragma once
#include <PxPhysicsAPI.h>

namespace sapien {
class SScene;

using namespace physx;

class DefaultEventCallback : public PxSimulationEventCallback {
  SScene *mScene;

public:
  void onContact(const PxContactPairHeader &pairHeader, const PxContactPair *pairs,
                 PxU32 nbPairs) override;

  void onAdvance(const PxRigidBody *const *bodyBuffer, const PxTransform *poseBuffer,
                 const PxU32 count) override;

  void onWake(PxActor **actors, PxU32 count) override;
  void onSleep(PxActor **actors, PxU32 count) override;
  void onConstraintBreak(PxConstraintInfo *constraints, PxU32 count) override;
  void onTrigger(PxTriggerPair *pairs, PxU32 count) override;

  DefaultEventCallback(SScene *scene);
};

} // namespace sapien
