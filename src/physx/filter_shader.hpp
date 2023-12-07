#pragma once
#include <PxFiltering.h>

using namespace physx;
namespace sapien {
namespace physx {

inline PxFilterFlags
TypeAffinityIgnoreFilterShader(PxFilterObjectAttributes attributes0, PxFilterData filterData0,
                               PxFilterObjectAttributes attributes1, PxFilterData filterData1,
                               PxPairFlags &pairFlags, const void *constantBlock,
                               PxU32 constantBlockSize) {

  if (PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1)) {
    pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
    return PxFilterFlag::eDEFAULT;
  }

  // if top 16 bits of word3 are different, the shapes will never collide
  // e.g. they are in different scenes
  if ((filterData0.word3 & 0xffff0000) != (filterData1.word3 & 0xffff0000)) {
    return PxFilterFlag::eKILL;
  }

  // if the lower 16 bits of word3 are the same (e.g. articulation id)
  // if word2 has a matching bit (e.g. door and frame both set the same bit)
  // the shapes will not collide (e.g. ignore collisions within each articulation)
  if ((filterData0.word2 & filterData1.word2) &&
      ((filterData0.word3 & 0xffff) == (filterData1.word3 & 0xffff))) {
    return PxFilterFlag::eKILL;
  }

  // Otherwise, apply MuJoCo's collision model to word0 and word1
  if ((filterData0.word0 & filterData1.word1) || (filterData1.word0 & filterData0.word1)) {
    pairFlags = PxPairFlag::eCONTACT_DEFAULT | PxPairFlag::eNOTIFY_CONTACT_POINTS |
                PxPairFlag::eNOTIFY_TOUCH_PERSISTS | PxPairFlag::eNOTIFY_TOUCH_FOUND |
                PxPairFlag::eNOTIFY_TOUCH_LOST | PxPairFlag::ePRE_SOLVER_VELOCITY |
                PxPairFlag::ePOST_SOLVER_VELOCITY | PxPairFlag::eDETECT_CCD_CONTACT;

    return PxFilterFlag::eDEFAULT;
  }
  return PxFilterFlag::eKILL;
}

} // namespace physx
} // namespace sapien
