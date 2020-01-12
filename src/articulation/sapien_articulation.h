#pragma once
#include "sapien_articulation_base.h"
#include <memory>

namespace sapien {
using namespace physx;

class SScene;
class SLink;
class SJoint;

class SArticulation : public SArticulationDrivable {
  friend class ArticulationBuilder;
  friend class LinkBuilder;

  SScene *mScene;

  PxArticulationReducedCoordinate *mPxArticulation = nullptr;
  PxArticulationCache *mCache = nullptr;

  std::vector<std::unique_ptr<SLink>> mLinks;
  std::vector<std::unique_ptr<SJoint>> mJoints;

  std::vector<uint32_t> mDofStarts;

  std::vector<uint32_t> mIndexE2I;
  std::vector<uint32_t> mIndexI2E;

public:
  std::vector<SLinkBase *> getLinks() override;
  std::vector<SJointBase *> getJoints() override;

  EArticulationType getType() const override;
  uint32_t dof() const override;

  std::vector<physx::PxReal> getQpos() const override;
  void setQpos(std::vector<physx::PxReal> const &v) override;

  std::vector<physx::PxReal> getQvel() const override;
  void setQvel(std::vector<physx::PxReal> const &v) override;

  std::vector<physx::PxReal> getQacc() const override;
  void setQacc(std::vector<physx::PxReal> const &v) override;

  std::vector<physx::PxReal> getQf() const override;
  void setQf(std::vector<physx::PxReal> const &v) override;

  std::vector<std::array<physx::PxReal, 2>> getQlimits() const override;
  void setQlimits(std::vector<std::array<physx::PxReal, 2>> const &v) const override;

  void setDriveTarget(std::vector<physx::PxReal> const &v) override;
  void setRootPose(physx::PxTransform const &T) override;

  inline PxArticulationReducedCoordinate *getPxArticulation() { return mPxArticulation; }

private:
  SArticulation(SScene *scene);
  SArticulation(SArticulation const &other) = delete;
  SArticulation &operator=(SArticulation const &other) = delete;

  std::vector<PxReal> E2I(std::vector<PxReal> ev) const;
  std::vector<PxReal> I2E(std::vector<PxReal> iv) const;
};

} // namespace sapien
