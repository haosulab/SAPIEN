#include "sapien/gear_joint.h"

using namespace physx;
namespace sapien {

PxConstraintShaderTable GearJoint::sShaderTable = {
    &GearJoint::solverPrep, &GearJoint::project, &GearJoint::visualize, PxConstraintFlag::Enum(0)};

GearJoint::GearJoint(physx::PxPhysics &physics, physx::PxRigidBody &body0,
                     const physx::PxTransform &localFrame0, physx::PxRigidBody &body1,
                     const physx::PxTransform &localFrame1) {
  mConstraint = physics.createConstraint(&body0, &body1, *this, GearJoint::sShaderTable,
                                         sizeof(GearJointData));
  mBody[0] = &body0;
  mBody[1] = &body1;

  mLocalPose[0] = localFrame0.getNormalized();
  mLocalPose[1] = localFrame1.getNormalized();

  mData.ratio = 1.0f;

  mData.c2b[0] = body0.getCMassLocalPose().transformInv(mLocalPose[0]);
  mData.c2b[1] = body0.getCMassLocalPose().transformInv(mLocalPose[1]);
}

void GearJoint::release() { mConstraint->release(); }

void GearJoint::setRatio(float ratio) {
  mData.ratio = ratio;
  mConstraint->markDirty();
}

float GearJoint::getRatio() const { return mData.ratio; }

///////////////////////////////////////////// PxConstraintConnector methods
void *GearJoint::prepareData() { return &mData; }

void GearJoint::onConstraintRelease() { delete this; }

void GearJoint::onComShift(PxU32 actor) {
  mData.c2b[actor] = mBody[actor]->getCMassLocalPose().transformInv(mLocalPose[actor]);
  mConstraint->markDirty();
}

void GearJoint::onOriginShift(const PxVec3 &shift) {}

void *GearJoint::getExternalReference(PxU32 &typeID) {
  typeID = TYPE_ID;
  return this;
}

///////////////////////////////////////////// work functions

PxU32 GearJoint::solverPrep(Px1DConstraint *constraints, PxVec3 &body0WorldOffset,
                            PxU32 maxConstraints, PxConstraintInvMassScale &,
                            const void *constantBlock, const PxTransform &bA2w,
                            const PxTransform &bB2w, bool /*useExtendedLimits*/, PxVec3 &cA2wOut,
                            PxVec3 &cB2wOut) {
  PX_UNUSED(maxConstraints);

  const GearJointData &data = *reinterpret_cast<const GearJointData *>(constantBlock);

  PxTransform cA2w = bA2w.transform(data.c2b[0]);
  PxTransform cB2w = bB2w.transform(data.c2b[1]);

  cA2wOut = cA2w.p;
  cB2wOut = cB2w.p;

  body0WorldOffset = cB2w.p - bA2w.p;

  const PxVec3 axis0 = cA2w.rotate(PxVec3(1.0f, 0.0f, 0.0f));
  const PxVec3 axis1 = cB2w.rotate(PxVec3(1.0f, 0.0f, 0.0f));

  Px1DConstraint *c = constraints;

  c->solveHint = PxConstraintSolveHint::eNONE;
  c->linear0 = PxVec3(0.0f);
  c->angular0 = -axis0;
  c->linear1 = PxVec3(0.0f);
  c->angular1 = -axis1 * data.ratio;
  c->velocityTarget = 0.0f;
  c->minImpulse = -PX_MAX_F32;
  c->maxImpulse = PX_MAX_F32;
  c->flags |= Px1DConstraintFlag::eANGULAR_CONSTRAINT;

  return 1;
}

void GearJoint::project(const void *constantBlock, PxTransform &bodyAToWorld,
                        PxTransform &bodyBToWorld, bool projectToA) {
  PX_UNUSED(constantBlock);
  PX_UNUSED(bodyAToWorld);
  PX_UNUSED(bodyBToWorld);
  PX_UNUSED(projectToA);
}

void GearJoint::visualize(PxConstraintVisualizer &viz, const void *constantBlock,
                          const PxTransform &body0Transform, const PxTransform &body1Transform,
                          PxU32 flags) {
  PX_UNUSED(flags);
  const GearJointData &data = *reinterpret_cast<const GearJointData *>(constantBlock);

  PxTransform cA2w = body0Transform * data.c2b[0];
  PxTransform cB2w = body1Transform * data.c2b[1];
  viz.visualizeJointFrames(cA2w, cB2w);
}

} // namespace sapien
