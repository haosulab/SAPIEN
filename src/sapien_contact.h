#pragma once
#include <vector>
#include <PxPhysicsAPI.h>

namespace sapien {
using namespace physx;

class SActorBase;

struct SContactPoint {
  PxVec3 position;
  PxVec3 normal;
  PxVec3 impulse;
  PxReal separation;
};

struct SContact {
  SActorBase *actors[2];
  bool starts;
  bool ends;
  bool persists;
  std::vector<SContactPoint> points;
};

} // namespace sapien
