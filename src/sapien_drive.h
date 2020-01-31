#pragma once
#include <PxPhysicsAPI.h>

namespace sapien {
class SScene;
class SActorBase;

using namespace physx;
class SDrive {
  friend SScene;

private:
  SScene *mScene;
  PxD6Joint *mJoint;
  SActorBase *actor1;
  SActorBase *actor2;

  SDrive(SScene *scene, SActorBase *actor1, PxTransform const &pose1, SActorBase *actor2,
         PxTransform const &pose2);

public:
  void setProperties(PxReal stiffness, PxReal damping, PxReal forceLimit, bool isAcceleration);

  void setTarget(PxTransform const &pose);
  void setTargetVelocity(PxVec3 const &velocity, PxVec3 const &angularVelocity);

  void destroy();
};
} // namespace sapien
