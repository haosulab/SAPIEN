#pragma once
#include <PxFiltering.h>

namespace sapien {
using namespace physx;

inline PxFilterFlags
TypeAffinityIgnoreFilterShader(PxFilterObjectAttributes attributes0, PxFilterData filterData0,
                               PxFilterObjectAttributes attributes1, PxFilterData filterData1,
                               PxPairFlags &pairFlags, const void *constantBlock,
                               PxU32 constantBlockSize) {

  if (PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1)) {
    pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
    return PxFilterFlag::eDEFAULT;
  }

  if ((filterData0.word2 & filterData1.word2) &&
      ((filterData0.word3 & 0xffff) == (filterData1.word3 & 0xffff))) {
    return PxFilterFlag::eKILL;
  }

  if ((filterData0.word0 & filterData1.word1) || (filterData1.word0 & filterData0.word1)) {
    pairFlags = PxPairFlag::eCONTACT_DEFAULT | PxPairFlag::eNOTIFY_CONTACT_POINTS |
                PxPairFlag::eNOTIFY_TOUCH_PERSISTS | PxPairFlag::eNOTIFY_TOUCH_FOUND |
                PxPairFlag::eNOTIFY_TOUCH_LOST | PxPairFlag::ePRE_SOLVER_VELOCITY |
                PxPairFlag::ePOST_SOLVER_VELOCITY | PxPairFlag::eDETECT_CCD_CONTACT;

    return PxFilterFlag::eDEFAULT;
  }
  return PxFilterFlag::eKILL;
}

} // namespace sapien
