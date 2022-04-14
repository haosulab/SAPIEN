#pragma once
#include <vector>
#include <PxPhysicsAPI.h>

namespace sapien {
using namespace physx;

class SActorBase;

struct STrigger {
  SActorBase *triggerActor;
  SActorBase *otherActor;
  bool starts;
  bool ends;
};

} // namespace sapien
