#pragma once
#include "sapien_articulation_base.h"
#include <memory>

namespace sapien {

class SScene;
class SKLink;
class SKJoint;

class SKArticulation : public SArticulationDrivable {
  friend class ArticulationBuilder;
  friend class LinkBuilder;

  std::vector<std::unique_ptr<SKLink>> mLinks;
  std::vector<std::unique_ptr<SKJoint>> mJoints;
  SKLink *mRootLink;

  uint32_t mDof;

  std::vector<int> mSortedIndices;

public:
  virtual std::vector<SLinkBase *> getBaseLinks() override;
  virtual std::vector<SJointBase *> getBaseJoints() override;
  virtual SLinkBase *getRootLink() const override;

  virtual EArticulationType getType() const override;
  virtual uint32_t dof() const override;

  virtual std::vector<physx::PxReal> getQpos() const override;
  virtual void setQpos(const std::vector<physx::PxReal> &v) override;

  virtual std::vector<physx::PxReal> getQvel() const override;
  virtual void setQvel(const std::vector<physx::PxReal> &v) override;

  virtual std::vector<physx::PxReal> getQacc() const override;
  virtual void setQacc(const std::vector<physx::PxReal> &v) override;

  virtual std::vector<physx::PxReal> getQf() const override;
  virtual void setQf(const std::vector<physx::PxReal> &v) override;
  virtual void setRootPose(const physx::PxTransform &T) override;

  virtual std::vector<std::array<physx::PxReal, 2>> getQlimits() const override;
  virtual void setQlimits(std::vector<std::array<physx::PxReal, 2>> const &v) const override;

  virtual void setDriveTarget(std::vector<physx::PxReal> const &v) override;
  virtual std::vector<physx::PxReal> getDriveTarget() const override;

  void prestep() override;

  SKArticulation(SKArticulation const &) = delete;
  SKArticulation &operator=(SKArticulation const &) = delete;
  ~SKArticulation() = default;

private:
  SKArticulation(SScene *scene);
};

} // namespace sapien
