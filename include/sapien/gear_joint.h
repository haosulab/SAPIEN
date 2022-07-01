#pragma once

#include <PxPhysicsAPI.h>

namespace sapien {

class GearJoint : public physx::PxConstraintConnector {
public:
  static const physx::PxU32 TYPE_ID = physx::PxConcreteType::eFIRST_USER_EXTENSION + 1;
  GearJoint(physx::PxPhysics &physics, physx::PxRigidBody &body0,
            const physx::PxTransform &localFrame0, physx::PxRigidBody &body1,
            const physx::PxTransform &localFrame1);

  void release();

  void setRatio(physx::PxReal ratio);
  physx::PxReal getRatio() const;

  // PxConstraintConnector boilerplate
  void *prepareData() override;
  void onConstraintRelease() override;
  void onComShift(physx::PxU32 actor) override;
  void onOriginShift(const physx::PxVec3 &shift) override;
  void *getExternalReference(physx::PxU32 &typeID) override;

  bool updatePvdProperties(physx::pvdsdk::PvdDataStream &pvdConnection,
                           const physx::PxConstraint *c,
                           physx::PxPvdUpdateType::Enum updateType) const override {
    return false;
  };
  physx::PxBase *getSerializable() override { return NULL; }
  physx::PxConstraintSolverPrep getPrep() const override { return sShaderTable.solverPrep; }

  const void *getConstantBlock() const override { return &mData; }

private:
  static physx::PxU32 solverPrep(physx::Px1DConstraint *constraints,
                                 physx::PxVec3 &body0WorldOffset, physx::PxU32 maxConstraints,
                                 physx::PxConstraintInvMassScale &, const void *constantBlock,
                                 const physx::PxTransform &bA2w, const physx::PxTransform &bB2w,
                                 bool useExtendedLimits, physx::PxVec3 &cA2wOut,
                                 physx::PxVec3 &cB2wOut);

  static void project(const void *constantBlock, physx::PxTransform &bodyAToWorld,
                      physx::PxTransform &bodyBToWorld, bool projectToA);

  static void visualize(physx::PxConstraintVisualizer &viz, const void *constantBlock,
                        const physx::PxTransform &body0Transform,
                        const physx::PxTransform &body1Transform, physx::PxU32 flags);

  struct GearJointData {
    physx::PxTransform c2b[2];
    physx::PxReal ratio;
  };

  physx::PxRigidBody *mBody[2];
  physx::PxTransform mLocalPose[2];

  physx::PxConstraint *mConstraint{};
  GearJointData mData;
  static physx::PxConstraintShaderTable sShaderTable;

public:
  ~GearJoint() = default;
};

} // namespace sapien
