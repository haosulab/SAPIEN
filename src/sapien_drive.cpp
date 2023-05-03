#include "sapien/sapien_drive.h"
#include "sapien/sapien_actor.h"
#include "sapien/sapien_scene.h"
#include "sapien/simulation.h"

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

std::vector<int> SDrive6D::getDofStates() {
  std::vector<int> dofState;
  dofState.push_back(mJoint->getMotion(PxD6Axis::eX));
  dofState.push_back(mJoint->getMotion(PxD6Axis::eY));
  dofState.push_back(mJoint->getMotion(PxD6Axis::eZ));
  dofState.push_back(mJoint->getMotion(PxD6Axis::eTWIST));
  dofState.push_back(mJoint->getMotion(PxD6Axis::eSWING1));
  dofState.push_back(mJoint->getMotion(PxD6Axis::eSWING2));
  return dofState;
}

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

std::vector<float> SDrive6D::getXLimit() const {
  std::vector<float> xLimit;
  xLimit.push_back(mJoint->getLinearLimit(PxD6Axis::eX).lower);
  xLimit.push_back(mJoint->getLinearLimit(PxD6Axis::eX).upper);
  return xLimit;
}

void SDrive6D::setYLimit(float low, float high) {
  mJoint->setMotion(PxD6Axis::eY, PxD6Motion::eLIMITED);
  mJoint->setLinearLimit(
      PxD6Axis::eY, PxJointLinearLimitPair(
                        mScene->getSimulation()->mPhysicsSDK->getTolerancesScale(), low, high));
}

std::vector<float> SDrive6D::getYLimit() const {
  std::vector<float> yLimit;
  yLimit.push_back(mJoint->getLinearLimit(PxD6Axis::eY).lower);
  yLimit.push_back(mJoint->getLinearLimit(PxD6Axis::eY).upper);
  return yLimit;
}

void SDrive6D::setZLimit(float low, float high) {
  mJoint->setMotion(PxD6Axis::eZ, PxD6Motion::eLIMITED);
  mJoint->setLinearLimit(
      PxD6Axis::eZ, PxJointLinearLimitPair(
                        mScene->getSimulation()->mPhysicsSDK->getTolerancesScale(), low, high));
}

std::vector<float> SDrive6D::getZLimit() const {
  std::vector<float> zLimit;
  zLimit.push_back(mJoint->getLinearLimit(PxD6Axis::eZ).lower);
  zLimit.push_back(mJoint->getLinearLimit(PxD6Axis::eZ).upper);
  return zLimit;
}

void SDrive6D::setXTwistLimit(float low, float high) {
  mJoint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);
  mJoint->setTwistLimit(PxJointAngularLimitPair(low, high));
}

std::vector<float> SDrive6D::getXTwistLimit() const {
  std::vector<float> xTwistLimit;
  xTwistLimit.push_back(mJoint->getTwistLimit().lower);
  xTwistLimit.push_back(mJoint->getTwistLimit().upper);
  return xTwistLimit;
}

void SDrive6D::setYZConeLimit(float yangle, float zangle) {
  mJoint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
  mJoint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLIMITED);
  mJoint->setSwingLimit(PxJointLimitCone(yangle, zangle));
}

std::vector<float> SDrive6D::getYZConeLimit() const {
  std::vector<float> yzConeLimit;
  yzConeLimit.push_back(mJoint->getSwingLimit().yAngle);
  yzConeLimit.push_back(mJoint->getSwingLimit().zAngle);
  return yzConeLimit;
}

void SDrive6D::setYZPyramidLimit(float yangleMin, float yangleMax, float zangleMin,
                                 float zangleMax) {
  mJoint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
  mJoint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLIMITED);
  mJoint->setPyramidSwingLimit(PxJointLimitPyramid(yangleMin, yangleMax, zangleMin, zangleMax));
}

std::vector<float> SDrive6D::getYZPyramidLimit() const {
  std::vector<float> yzPyramidLimit;
  yzPyramidLimit.push_back(mJoint->getPyramidSwingLimit().yAngleMin);
  yzPyramidLimit.push_back(mJoint->getPyramidSwingLimit().yAngleMax);
  yzPyramidLimit.push_back(mJoint->getPyramidSwingLimit().zAngleMin);
  yzPyramidLimit.push_back(mJoint->getPyramidSwingLimit().zAngleMax);
  return yzPyramidLimit;
}

void SDrive6D::setDistanceLimit(float dist) {
  mJoint->setMotion(PxD6Axis::eX, PxD6Motion::eLIMITED);
  mJoint->setMotion(PxD6Axis::eY, PxD6Motion::eLIMITED);
  mJoint->setMotion(PxD6Axis::eZ, PxD6Motion::eLIMITED);
  mJoint->setDistanceLimit(
      PxJointLinearLimit(mScene->getSimulation()->mPhysicsSDK->getTolerancesScale(), dist));
}

float SDrive6D::getDistanceLimit() const {
  return mJoint->getDistanceLimit().value;
}

void SDrive6D::setXProperties(PxReal stiffness, PxReal damping, PxReal forceLimit,
                              bool isAcceleration) {
  PxD6JointDrive drive(stiffness, damping, forceLimit, isAcceleration);
  mJoint->setDrive(PxD6Drive::eX, drive);
}

std::vector<float> SDrive6D::getXProperties() const {
  std::vector<float> properties;
  PxD6JointDrive drive = mJoint->getDrive(PxD6Drive::eX);
  properties.push_back(drive.stiffness);
  properties.push_back(drive.damping);
  properties.push_back(drive.forceLimit);
  if (drive.flags.isSet(PxD6JointDriveFlag::eACCELERATION)) {
    properties.push_back(1.0); // isAcceleration true
  } else {
    properties.push_back(0.0); // isAcceleration false
  }
  return properties;
}

void SDrive6D::setYProperties(PxReal stiffness, PxReal damping, PxReal forceLimit,
                              bool isAcceleration) {
  PxD6JointDrive drive(stiffness, damping, forceLimit, isAcceleration);
  mJoint->setDrive(PxD6Drive::eY, drive);
}

std::vector<float> SDrive6D::getYProperties() const {
  std::vector<float> properties;
  PxD6JointDrive drive = mJoint->getDrive(PxD6Drive::eY);
  properties.push_back(drive.stiffness);
  properties.push_back(drive.damping);
  properties.push_back(drive.forceLimit);
  if (drive.flags.isSet(PxD6JointDriveFlag::eACCELERATION)) {
    properties.push_back(1.0); // isAcceleration true
  } else {
    properties.push_back(0.0); // isAcceleration false
  }
  return properties;
}

void SDrive6D::setZProperties(PxReal stiffness, PxReal damping, PxReal forceLimit,
                              bool isAcceleration) {
  PxD6JointDrive drive(stiffness, damping, forceLimit, isAcceleration);
  mJoint->setDrive(PxD6Drive::eZ, drive);
}

std::vector<float> SDrive6D::getZProperties() const {
  std::vector<float> properties;
  PxD6JointDrive drive = mJoint->getDrive(PxD6Drive::eZ);
  properties.push_back(drive.stiffness);
  properties.push_back(drive.damping);
  properties.push_back(drive.forceLimit);
  if (drive.flags.isSet(PxD6JointDriveFlag::eACCELERATION)) {
    properties.push_back(1.0); // isAcceleration true
  } else {
    properties.push_back(0.0); // isAcceleration false
  }
  return properties;
}

void SDrive6D::setXTwistDriveProperties(PxReal stiffness, PxReal damping, PxReal forceLimit,
                                        bool isAcceleration) {
  PxD6JointDrive drive(stiffness, damping, forceLimit, isAcceleration);
  mJoint->setDrive(PxD6Drive::eTWIST, drive);
}

std::vector<float> SDrive6D::getXTwistDriveProperties() const {
  std::vector<float> properties;
  PxD6JointDrive drive = mJoint->getDrive(PxD6Drive::eTWIST);
  properties.push_back(drive.stiffness);
  properties.push_back(drive.damping);
  properties.push_back(drive.forceLimit);
  if (drive.flags.isSet(PxD6JointDriveFlag::eACCELERATION)) {
    properties.push_back(1.0); // isAcceleration true
  } else {
    properties.push_back(0.0); // isAcceleration false
  }
  return properties;
}

void SDrive6D::setYZSwingDriveProperties(PxReal stiffness, PxReal damping, PxReal forceLimit,
                                         bool isAcceleration) {
  PxD6JointDrive drive(stiffness, damping, forceLimit, isAcceleration);
  mJoint->setDrive(PxD6Drive::eSWING, drive);
}

std::vector<float> SDrive6D::getYZSwingDriveProperties() const {
  std::vector<float> properties;
  PxD6JointDrive drive = mJoint->getDrive(PxD6Drive::eSWING);
  properties.push_back(drive.stiffness);
  properties.push_back(drive.damping);
  properties.push_back(drive.forceLimit);
  if (drive.flags.isSet(PxD6JointDriveFlag::eACCELERATION)) {
    properties.push_back(1.0); // isAcceleration true
  } else {
    properties.push_back(0.0); // isAcceleration false
  }
  return properties;
}

void SDrive6D::setSlerpProperties(PxReal stiffness, PxReal damping, PxReal forceLimit,
                                  bool isAcceleration) {
  PxD6JointDrive drive(stiffness, damping, forceLimit, isAcceleration);
  mJoint->setDrive(PxD6Drive::eSLERP, drive);
}

std::vector<float> SDrive6D::getSlerpProperties() const {
  std::vector<float> properties;
  PxD6JointDrive drive = mJoint->getDrive(PxD6Drive::eSLERP);
  properties.push_back(drive.stiffness);
  properties.push_back(drive.damping);
  properties.push_back(drive.forceLimit);
  if (drive.flags.isSet(PxD6JointDriveFlag::eACCELERATION)) {
    properties.push_back(1.0); // isAcceleration true
  } else {
    properties.push_back(0.0); // isAcceleration false
  }
  return properties;
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

SGear::SGear(SScene *scene, SActorBase *actor1, PxTransform const &pose1, SActorBase *actor2,
             PxTransform const &pose2)
    : SDrive(scene, actor1, actor2) {
  PxRigidActor *pxa1 = actor1 ? actor1->getPxActor() : nullptr;
  PxRigidActor *pxa2 = actor2 ? actor2->getPxActor() : nullptr;
  mJoint = PxGearJointCreate(*scene->getSimulation()->mPhysicsSDK, pxa1, pose1, pxa2, pose2);
}

PxJoint *SGear::getPxJoint() const { return mJoint; }
float SGear::getGearRatio() const { return mJoint->getGearRatio(); }
void SGear::setGearRatio(float ratio) { mJoint->setGearRatio(ratio); }

} // namespace sapien
