#pragma once
#include <PxPhysicsAPI.h>
#include <tuple>

namespace sapien {
class SScene;
class SActorBase;

using namespace physx;
class SDrive {
  friend SScene;

private:
  SScene *mScene;
  PxD6Joint *mJoint;
  SActorBase *mActor1;
  SActorBase *mActor2;

  SDrive(SScene *scene, SActorBase *actor1, PxTransform const &pose1, SActorBase *actor2,
         PxTransform const &pose2);

public:
  inline SActorBase *getActor1() const { return mActor1; };
  inline SActorBase *getActor2() const { return mActor2; };

  PxTransform getLocalPose1() const;
  PxTransform getLocalPose2() const;

  void setProperties(PxReal stiffness, PxReal damping, PxReal forceLimit, bool isAcceleration);

  void setTarget(PxTransform const &pose);
  PxTransform getTarget() const;

  void setTargetVelocity(PxVec3 const &velocity, PxVec3 const &angularVelocity);
  std::tuple<PxVec3, PxVec3> getTargetVelocity() const;

  void destroy();
};
} // namespace sapien
