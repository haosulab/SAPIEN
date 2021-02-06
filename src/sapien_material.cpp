#include "sapien_material.h"

namespace sapien {
SPhysicalMaterial::SPhysicalMaterial(physx::PxMaterial *material) { mMaterial = material; }
SPhysicalMaterial::~SPhysicalMaterial() { mMaterial->release(); }

} // namespace sapien
