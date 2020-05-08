#pragma once
#include <vector>
#include <PxPhysicsAPI.h>

namespace sapien {
using namespace physx;

class SActorBase;

struct SContact {
  SActorBase *actors[2];
  PxVec3 point;
  PxVec3 normal;
  PxVec3 impulse;
  PxReal separation;
  bool starts;
  bool ends;
  bool persists;
};

} // namespace sapien
