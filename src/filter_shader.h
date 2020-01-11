#pragma once

#include <PxFiltering.h>
#include <algorithm>
#include <iostream>
#include <map>
#include <set>

namespace sapien {
using namespace physx;

inline PxFilterFlags TypeAffinityFilterShader(PxFilterObjectAttributes attributes0,
                                              PxFilterData filterData0,
                                              PxFilterObjectAttributes attributes1,
                                              PxFilterData filterData1, PxPairFlags &pairFlags,
                                              const void *constantBlock, PxU32 constantBlockSize) {

  if (PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1)) {
    pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
    return PxFilterFlag::eDEFAULT;
  }

  pairFlags = PxPairFlag::eCONTACT_DEFAULT | PxPairFlag::eNOTIFY_TOUCH_PERSISTS |
              PxPairFlag::eNOTIFY_CONTACT_POINTS;
  if ((filterData0.word0 & filterData1.word1) || (filterData1.word0 & filterData0.word1)) {
    return PxFilterFlag::eDEFAULT;
  }
  return PxFilterFlag::eKILL;
}

// TODO: test this class
// class CollisionGroupManager {
//   std::set<int> exclusiveGroups;
//   std::map<int, std::set<int>> reusableGroups;
//   int count = 0;

// public:
//   CollisionGroupManager() {}
//   int NewExclusiveGroup();
//   int NewReusableGroup(int id);
//   static void addGroupToData(PxFilterData &data, int group);
// };

} // namespace sapien
