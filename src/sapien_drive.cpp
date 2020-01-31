#include "sapien_drive.h"
#include "sapien_actor.h"
#include "sapien_scene.h"
#include "simulation.h"

namespace sapien {

SDrive::SDrive(SScene *scene, SActorBase *actor1, PxTransform const &pose1, SActorBase *actor2,
               PxTransform const &pose2) : mScene(scene) {
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

void SDrive::setProperties(PxReal stiffness, PxReal damping, PxReal forceLimit,
                           bool isAcceleration) {
  PxD6JointDrive drive(stiffness, damping, PX_MAX_F32, true);
  mJoint->setDrive(PxD6Drive::eX, drive);
  mJoint->setDrive(PxD6Drive::eY, drive);
  mJoint->setDrive(PxD6Drive::eZ, drive);
  mJoint->setDrive(PxD6Drive::eSLERP, drive);
}

void SDrive::setTarget(PxTransform const &pose) { mJoint->setDrivePosition(pose); }

void SDrive::setTargetVelocity(PxVec3 const &velocity, PxVec3 const &angularVelocity) {
  mJoint->setDriveVelocity(velocity, angularVelocity);
}

void SDrive::destroy() {
  mScene->removeDrive(this);
}

} // namespace sapien
