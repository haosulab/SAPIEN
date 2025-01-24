/*
 * Copyright 2025 Hillbot Inc.
 * Copyright 2020-2024 UCSD SU Lab
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
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

inline PxFilterFlags
TypeAffinityIgnoreFilterShaderGpu(PxFilterObjectAttributes attributes0, PxFilterData filterData0,
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
    pairFlags = PxPairFlag::eCONTACT_DEFAULT;
    return PxFilterFlag::eDEFAULT;
  }
  return PxFilterFlag::eKILL;
}

} // namespace physx
} // namespace sapien
