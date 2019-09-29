#include "my_filter_shader.h"
using namespace physx;

PxFilterFlags MyFilterShader
(PxFilterObjectAttributes attributes0, PxFilterData filterData0, 
 PxFilterObjectAttributes attributes1, PxFilterData filterData1,
 PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
	pairFlags = PxPairFlag::eCONTACT_DEFAULT;
	pairFlags |= PxPairFlags(PxU16(filterData0.word2 | filterData1.word2));

	return PxFilterFlags();
}