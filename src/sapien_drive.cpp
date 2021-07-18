#include "sapien_drive.h"
#include "sapien_actor.h"
#include "sapien_scene.h"
#include "simulation.h"

namespace sapien {

PxTransform SDrive::getLocalPose1() const {
  return getPxJoint()->getLocalPose(PxJointActorIndex::eACTOR0);
}

PxTransform SDrive::getLocalPose2() const {
  return getPxJoint()->getLocalPose(PxJointActorIndex::eACTOR1);
}

SDrive::SDrive(SScene *scene, SActorBase *actor1, SActorBase *actor2)
    : mScene(scene), mActor1(actor1), mActor2(actor2){};

SDrive6D::SDrive6D(SScene *scene, SActorBase *actor1, PxTransform const &pose1, SActorBase *actor2,
                   PxTransform const &pose2)
    : SDrive(scene, actor1, actor2) {
  PxRigidActor *pxa1 = actor1 ? actor1->getPxActor() : nullptr;
  PxRigidActor *pxa2 = actor2 ? actor2->getPxActor() : nullptr;
  mJoint = PxD6JointCreate(*scene->getSimulation()->mPhysicsSDK, pxa1, pose1, pxa2, pose2);
  mJoint->userData = this;
  mJoint->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
  mJoint->setMotion(PxD6Axis::eY, PxD6Motion::eFREE);
  mJoint->setMotion(PxD6Axis::eZ, PxD6Motion::eFREE);
  mJoint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
  mJoint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
  mJoint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
}

PxJoint *SDrive6D::getPxJoint() const { return mJoint; }

void SDrive6D::lockMotion(bool tx, bool ty, bool tz, bool rx, bool ry, bool rz) {
  if (tx) {
    mJoint->setMotion(PxD6Axis::eX, PxD6Motion::eLOCKED);
  }
  if (ty) {
    mJoint->setMotion(PxD6Axis::eY, PxD6Motion::eLOCKED);
  }
  if (tz) {
    mJoint->setMotion(PxD6Axis::eZ, PxD6Motion::eLOCKED);
  }
  if (rx) {
    mJoint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLOCKED);
  }
  if (ry) {
    mJoint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLOCKED);
  }
  if (rz) {
    mJoint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLOCKED);
  }
}

void SDrive6D::freeMotion(bool tx, bool ty, bool tz, bool rx, bool ry, bool rz) {
  if (tx) {
    mJoint->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
  }
  if (ty) {
    mJoint->setMotion(PxD6Axis::eY, PxD6Motion::eFREE);
  }
  if (tz) {
    mJoint->setMotion(PxD6Axis::eZ, PxD6Motion::eFREE);
  }
  if (rx) {
    mJoint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
  }
  if (ry) {
    mJoint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
  }
  if (rz) {
    mJoint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
  }
}

void SDrive6D::setXLimit(float low, float high) {
  mJoint->setMotion(PxD6Axis::eX, PxD6Motion::eLIMITED);
  mJoint->setLinearLimit(
      PxD6Axis::eX, PxJointLinearLimitPair(
                        mScene->getSimulation()->mPhysicsSDK->getTolerancesScale(), low, high));
}

void SDrive6D::setYLimit(float low, float high) {
  mJoint->setMotion(PxD6Axis::eY, PxD6Motion::eLIMITED);
  mJoint->setLinearLimit(
      PxD6Axis::eY, PxJointLinearLimitPair(
                        mScene->getSimulation()->mPhysicsSDK->getTolerancesScale(), low, high));
}

void SDrive6D::setZLimit(float low, float high) {
  mJoint->setMotion(PxD6Axis::eZ, PxD6Motion::eLIMITED);
  mJoint->setLinearLimit(
      PxD6Axis::eZ, PxJointLinearLimitPair(
                        mScene->getSimulation()->mPhysicsSDK->getTolerancesScale(), low, high));
}

void SDrive6D::setXTwistLimit(float low, float high) {
  mJoint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);
  mJoint->setTwistLimit(PxJointAngularLimitPair(low, high));
}

void SDrive6D::setYZConeLimit(float yangle, float zangle) {
  mJoint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
  mJoint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLIMITED);
  mJoint->setSwingLimit(PxJointLimitCone(yangle, zangle));
}

void SDrive6D::setYZPyramidLimit(float yangleMin, float yangleMax, float zangleMin,
                                 float zangleMax) {
  mJoint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
  mJoint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLIMITED);
  mJoint->setPyramidSwingLimit(PxJointLimitPyramid(yangleMin, yangleMax, zangleMin, zangleMax));
}

void SDrive6D::setDistanceLimit(float dist) {
  mJoint->setMotion(PxD6Axis::eX, PxD6Motion::eLIMITED);
  mJoint->setMotion(PxD6Axis::eY, PxD6Motion::eLIMITED);
  mJoint->setMotion(PxD6Axis::eZ, PxD6Motion::eLIMITED);
  mJoint->setDistanceLimit(
      PxJointLinearLimit(mScene->getSimulation()->mPhysicsSDK->getTolerancesScale(), dist));
}

void SDrive6D::setXProperties(PxReal stiffness, PxReal damping, PxReal forceLimit,
                              bool isAcceleration) {
  PxD6JointDrive drive(stiffness, damping, forceLimit, isAcceleration);
  mJoint->setDrive(PxD6Drive::eX, drive);
}

void SDrive6D::setYProperties(PxReal stiffness, PxReal damping, PxReal forceLimit,
                              bool isAcceleration) {
  PxD6JointDrive drive(stiffness, damping, forceLimit, isAcceleration);
  mJoint->setDrive(PxD6Drive::eY, drive);
}
void SDrive6D::setZProperties(PxReal stiffness, PxReal damping, PxReal forceLimit,
                              bool isAcceleration) {
  PxD6JointDrive drive(stiffness, damping, forceLimit, isAcceleration);
  mJoint->setDrive(PxD6Drive::eZ, drive);
}

void SDrive6D::setXTwistDriveProperties(PxReal stiffness, PxReal damping, PxReal forceLimit,
                                        bool isAcceleration) {
  PxD6JointDrive drive(stiffness, damping, forceLimit, isAcceleration);
  mJoint->setDrive(PxD6Drive::eTWIST, drive);
}

void SDrive6D::setYZSwingDriveProperties(PxReal stiffness, PxReal damping, PxReal forceLimit,
                                         bool isAcceleration) {
  PxD6JointDrive drive(stiffness, damping, forceLimit, isAcceleration);
  mJoint->setDrive(PxD6Drive::eSWING, drive);
}

void SDrive6D::setSlerpProperties(PxReal stiffness, PxReal damping, PxReal forceLimit,
                                  bool isAcceleration) {
  PxD6JointDrive drive(stiffness, damping, forceLimit, isAcceleration);
  mJoint->setDrive(PxD6Drive::eSLERP, drive);
}

void SDrive6D::setTarget(PxTransform const &pose) { mJoint->setDrivePosition(pose); }
PxTransform SDrive6D::getTarget() const { return mJoint->getDrivePosition(); }

void SDrive6D::setTargetVelocity(PxVec3 const &velocity, PxVec3 const &angularVelocity) {
  mJoint->setDriveVelocity(velocity, angularVelocity);
}

std::tuple<PxVec3, PxVec3> SDrive6D::getTargetVelocity() const {
  PxVec3 linear, angular;
  mJoint->getDriveVelocity(linear, angular);
  return {linear, angular};
}

/// revolute
SDriveRevolute::SDriveRevolute(SScene *scene, SActorBase *actor1, PxTransform const &pose1,
                               SActorBase *actor2, PxTransform const &pose2)
    : SDrive(scene, actor1, actor2) {
  PxRigidActor *pxa1 = actor1 ? actor1->getPxActor() : nullptr;
  PxRigidActor *pxa2 = actor2 ? actor2->getPxActor() : nullptr;

  mJoint = PxRevoluteJointCreate(*scene->getSimulation()->mPhysicsSDK, pxa1, pose1, pxa2, pose2);
  mJoint->userData = this;
  mJoint->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, true);
}

PxJoint *SDriveRevolute::getPxJoint() const { return mJoint; }

void SDriveRevolute::setProperties(PxReal stiffness, PxReal damping, PxReal forceLimit,
                                   bool isAcceleration) {
  mJoint->setDriveForceLimit(forceLimit);
}

void SDriveRevolute::removeLimit() {
  mJoint->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, false);
}

void SDriveRevolute::setLimit(float low, float high) {
  mJoint->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
  auto limit = mJoint->getLimit();
  limit.lower = low;
  limit.upper = high;
  limit.contactDistance = 0.1;
  mJoint->setLimit(limit);
}

PxVec2 SDriveRevolute::getLimit() const {
  if (!(mJoint->getRevoluteJointFlags() & PxRevoluteJointFlag::eLIMIT_ENABLED)) {
    return {-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()};
  }
  auto limit = mJoint->getLimit();
  return {limit.lower, limit.upper};
}

void SDriveRevolute::setLimitProperties(float stiffness, float damping) {
  auto limit = mJoint->getLimit();
  limit.stiffness = stiffness;
  limit.damping = damping;
  mJoint->setLimit(limit);
}

float SDriveRevolute::getLimitStiffness() const { return mJoint->getLimit().stiffness; }
float SDriveRevolute::getLimitDamping() const { return mJoint->getLimit().damping; }

/// prismatic

} // namespace sapien
