#include "PxPhysicsAPI.h"

using namespace physx;

enum
{

};

PxFilterFlags MyFilterShader
(PxFilterObjectAttributes attributes0, PxFilterData filterData0, 
 PxFilterObjectAttributes attributes1, PxFilterData filterData1,
 PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize);
