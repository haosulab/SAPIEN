#pragma once
#include "sapien_articulation_base.h"
#include <Eigen/Dense>
#include <memory>

namespace sapien {
using namespace physx;
using namespace Eigen;

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
  SLink *mRootLink = nullptr;

  std::vector<uint32_t> mIndexE2I;
  std::vector<uint32_t> mIndexI2E;

  /* Due to the capacity of matrix, cache the permutation matrix in advance */
  Matrix<PxReal, Dynamic, Dynamic, RowMajor> mColumnPermutationI2E;
  Matrix<PxReal, Dynamic, Dynamic, RowMajor> mRowPermutationI2E;

public:
  std::vector<SLinkBase *> getBaseLinks() override;
  std::vector<SJointBase *> getBaseJoints() override;

  std::vector<SLink *> getSLinks();
  std::vector<SJoint *> getSJoints();
  std::vector<SJoint *> getActiveJoints();

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

  std::vector<physx::PxReal> getDriveTarget() const override;
  void setDriveTarget(std::vector<physx::PxReal> const &v) override;

  void setRootPose(physx::PxTransform const &T) override;
  void setRootVelocity(physx::PxVec3 const &v);
  void setRootAngularVelocity(physx::PxVec3 const &omega);

  void prestep() override;

  SLinkBase *getRootLink() override;

  inline PxArticulationReducedCoordinate *getPxArticulation() { return mPxArticulation; }

  void resetCache();

  /* Dynamics Functions */
  std::vector<physx::PxReal> computePassiveForce(bool gravity = true,
                                                 bool coriolisAndCentrifugal = true,
                                                 bool external = true);
  std::vector<physx::PxReal> computeInverseDynamics(const std::vector<PxReal> &qacc);
  std::vector<physx::PxReal> computeForwardDynamics(const std::vector<PxReal> &qf);
  Matrix<PxReal, Dynamic, Dynamic, RowMajor> computeManipulatorInertiaMatrix();

  /* Kinematics Functions */
  Matrix<PxReal, Dynamic, Dynamic, RowMajor> computeWorldCartesianJacobianMatrix();
  Matrix<PxReal, Dynamic, Dynamic, RowMajor> computeSpatialTwistJacobianMatrix();

  static Matrix<PxReal, 4, 4, RowMajor> computeRelativeTransformation(SLink *sourceFrame,
                                                                      SLink *targetFrame);
  Matrix<PxReal, 4, 4, RowMajor> computeRelativeTransformation(uint32_t sourceLinkId,
                                                               uint32_t targetLinkId);
  static Matrix<PxReal, 6, 6, RowMajor> computeAdjointMatrix(SLink *sourceFrame,
                                                             SLink *targetFrame);
  Matrix<PxReal, 6, 6, RowMajor> computeAdjointMatrix(uint32_t sourceLinkId,
                                                      uint32_t targetLinkId);

  Matrix<PxReal, Dynamic, 1> computeTwistDiffIK(const Eigen::Matrix<PxReal, 6, 1> &spatialTwist,
                                                uint32_t commandedLinkId,
                                                const std::vector<uint32_t> &activeQIds = {});

  Matrix<PxReal, Dynamic, 1>
  computeCartesianVelocityDiffIK(const Eigen::Matrix<PxReal, 6, 1> &cartesianVelocity,
                                 uint32_t commandedLinkId,
                                 const std::vector<uint32_t> &activeQIds = {});
  /* Save and Load */
  std::vector<PxReal> packData();
  void unpackData(std::vector<PxReal> const &data);

private:
  SArticulation(SScene *scene);
  SArticulation(SArticulation const &other) = delete;
  SArticulation &operator=(SArticulation const &other) = delete;

  std::vector<PxReal> E2I(std::vector<PxReal> ev) const;
  std::vector<PxReal> I2E(std::vector<PxReal> iv) const;

  /* Functions for building permutation matrix for Jacobian calculation */
  static Matrix<PxReal, Dynamic, Dynamic, RowMajor>
  buildColumnPermutation(const std::vector<uint32_t> &indexI2E);
  Matrix<PxReal, Dynamic, Dynamic, RowMajor> buildRowPermutation();
};

} // namespace sapien
