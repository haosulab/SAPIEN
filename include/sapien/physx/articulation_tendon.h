#pragma once
#include "./articulation.h"
#include <PxPhysicsAPI.h>

namespace sapien {
namespace physx {

class PhysxArticulationSpatialTendon {
public:
  PhysxArticulationSpatialTendon(std::shared_ptr<PhysxArticulation> articulation,
                                 std::vector<std::shared_ptr<PhysxArticulationLinkComponent>> const &links,
                                 std::vector<int> const &parents, std::vector<float> const &coefficients,
                                 std::vector<Vec3> const &linkOffsets, std::vector<float> const &restLengths,
                                 std::vector<std::tuple<float, float>> limits, float stiffness, float damping,
                                 float limitStiffness);

  float getOffset() const;
  void setOffset(float offset);

  float getStiffness() const;
  void setStiffness(float stiffness);

  float getDamping() const;
  void setDamping(float damping);

  std::vector<std::shared_ptr<PhysxArticulationLinkComponent>> getLinks() const;
  std::vector<int> getLinkParents() const;
  std::vector<float> getCoefficients() const;
  std::vector<Vec3> getOffsets() const;

private:
  std::weak_ptr<PhysxArticulation> mArticulation;
  std::vector<std::weak_ptr<PhysxArticulationLinkComponent>> mLinks;
  std::vector<int> mLinkParents;
  ::physx::PxArticulationSpatialTendon *mTendon{};
};

} // namespace physx
} // namespace sapien
