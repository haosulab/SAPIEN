#include "sapien_drive.h"
#include "sapien_actor.h"
#include "sapien_scene.h"
#include "simulation.h"

namespace sapien {

SDrive::SDrive(SScene *scene, SActorBase *actor1, PxTransform const &pose1, SActorBase *actor2,
               PxTransform const &pose2)
    : mScene(scene), mActor1(actor1), mActor2(actor2) {
  PxRigidActor *pxa1 = actor1 ? actor1->getPxActor() : nullptr;
  PxRigidActor *pxa2 = actor2 ? actor2->getPxActor() : nullptr;

  mJoint = PxD6JointCreate(*scene->getEngine()->mPhysicsSDK, pxa1, pose1, pxa2, pose2);
  mJoint->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
  mJoint->setMotion(PxD6Axis::eY, PxD6Motion::eFREE);
  mJoint->setMotion(PxD6Axis::eZ, PxD6Motion::eFREE);
  mJoint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
  mJoint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
  mJoint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
}

void SDrive::lockMotion(bool tx, bool ty, bool tz, bool rx, bool ry, bool rz) {
  mJoint->setMotion(PxD6Axis::eX, tx ? PxD6Motion::eLOCKED : PxD6Motion::eFREE);
  mJoint->setMotion(PxD6Axis::eY, ty ? PxD6Motion::eLOCKED : PxD6Motion::eFREE);
  mJoint->setMotion(PxD6Axis::eZ, tz ? PxD6Motion::eLOCKED : PxD6Motion::eFREE);
  mJoint->setMotion(PxD6Axis::eTWIST, rx ? PxD6Motion::eLOCKED : PxD6Motion::eFREE);
  mJoint->setMotion(PxD6Axis::eSWING1, ry ? PxD6Motion::eLOCKED : PxD6Motion::eFREE);
  mJoint->setMotion(PxD6Axis::eSWING2, rz ? PxD6Motion::eLOCKED : PxD6Motion::eFREE);
}

void SDrive::setProperties(PxReal stiffness, PxReal damping, PxReal forceLimit,
                           bool isAcceleration) {
  PxD6JointDrive drive(stiffness, damping, forceLimit, isAcceleration);
  mJoint->setDrive(PxD6Drive::eX, drive);
  mJoint->setDrive(PxD6Drive::eY, drive);
  mJoint->setDrive(PxD6Drive::eZ, drive);
  mJoint->setDrive(PxD6Drive::eSLERP, drive);
}

void SDrive::setXProperties(PxReal stiffness, PxReal damping, PxReal forceLimit,
                            bool isAcceleration) {
  PxD6JointDrive drive(stiffness, damping, forceLimit, isAcceleration);
  mJoint->setDrive(PxD6Drive::eX, drive);
}

void SDrive::setYProperties(PxReal stiffness, PxReal damping, PxReal forceLimit,
                            bool isAcceleration) {
  PxD6JointDrive drive(stiffness, damping, forceLimit, isAcceleration);
  mJoint->setDrive(PxD6Drive::eY, drive);
}
void SDrive::setZProperties(PxReal stiffness, PxReal damping, PxReal forceLimit,
                            bool isAcceleration) {
  PxD6JointDrive drive(stiffness, damping, forceLimit, isAcceleration);
  mJoint->setDrive(PxD6Drive::eZ, drive);
}
void SDrive::setRotationProperties(PxReal stiffness, PxReal damping, PxReal forceLimit,
                                   bool isAcceleration) {
  PxD6JointDrive drive(stiffness, damping, forceLimit, isAcceleration);
  mJoint->setDrive(PxD6Drive::eSLERP, drive);
}

PxTransform SDrive::getLocalPose1() const {
  return mJoint->getLocalPose(PxJointActorIndex::eACTOR0);
}
PxTransform SDrive::getLocalPose2() const {
  return mJoint->getLocalPose(PxJointActorIndex::eACTOR1);
}

void SDrive::setTarget(PxTransform const &pose) { mJoint->setDrivePosition(pose); }
PxTransform SDrive::getTarget() const { return mJoint->getDrivePosition(); }

void SDrive::setTargetVelocity(PxVec3 const &velocity, PxVec3 const &angularVelocity) {
  mJoint->setDriveVelocity(velocity, angularVelocity);
}
std::tuple<PxVec3, PxVec3> SDrive::getTargetVelocity() const {
  PxVec3 linear, angular;
  mJoint->getDriveVelocity(linear, angular);
  return {linear, angular};
}

void SDrive::destroy() { mScene->removeDrive(this); }

} // namespace sapien
