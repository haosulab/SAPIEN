#pragma once
#include "sapien/gear_joint.h"
#include <memory>

namespace sapien {

class SScene;
class SActorDynamicBase;
using namespace physx;

class SGear {
  friend SScene;

protected:
  SScene *mScene;
  SActorDynamicBase *mActor1;
  SActorDynamicBase *mActor2;
  GearJoint *mGearJoint;

public:
  SGear(SScene *scene, SActorDynamicBase *actor1, PxTransform const &pose1,
        SActorDynamicBase *actor2, PxTransform const &pose2);
  inline GearJoint *getGearJoint() const { return mGearJoint; };

  inline SActorDynamicBase *getActor1() const { return mActor1; }
  inline SActorDynamicBase *getActor2() const { return mActor2; }

  float getRatio() const;
  void setRatio(float ratio);
  ~SGear() = default;
};
} // namespace sapien
