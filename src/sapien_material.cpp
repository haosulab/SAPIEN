#include "sapien/sapien_material.h"
#include "sapien/simulation.h"

namespace sapien {
SPhysicalMaterial::SPhysicalMaterial(std::shared_ptr<Simulation const> simulation,
                                     physx::PxMaterial *material)
    : mMaterial(material), mSimulation(simulation) {
  mMaterial->userData = this;
}
SPhysicalMaterial::~SPhysicalMaterial() { mMaterial->release(); }

} // namespace sapien
