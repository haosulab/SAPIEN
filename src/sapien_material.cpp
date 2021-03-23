#include "sapien_material.h"

namespace sapien {
SPhysicalMaterial::SPhysicalMaterial(physx::PxMaterial *material) : mMaterial(material) {
  mMaterial->userData = this;
}
SPhysicalMaterial::~SPhysicalMaterial() { mMaterial->release(); }

} // namespace sapien
