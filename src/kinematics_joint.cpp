//
// Created by sim on 9/26/19.
//

#include "kinematics_joint.h"
#include <assert.h>
#include <iostream>
using namespace physx;

// KJoint
KJoint::KJoint(PxRigidDynamic *childLink, KJoint *parentJoint, const PxTransform &toChild,
               const PxTransform &fromParent)
    : childLink(childLink), poseFromParent(fromParent), poseToChild(toChild), parent(parentJoint) {
  parentLink = parentJoint ? parentJoint->childLink : nullptr;
  if (parentJoint) {
    parentJoint->children.push_back(this);
  }
}
PxTransform KJoint::passThrough(const PxTransform &T) {
  PxTransform parentPose(PxIdentity);
  if (parentLink) {
    parentLink->getKinematicTarget(parentPose);
  }
  PxTransform toSet = parentPose * poseFromParent * T * poseToChild;
  return toSet;
}

// RevoluteKJoint
RevoluteKJoint::RevoluteKJoint(PxRigidDynamic *childLink, KJoint *parentJoint,
                               const PxTransform &toChild, const PxTransform &fromParent,
                               PxReal upperLimit, PxReal lowerLimit)
    : SingleDOFKJoint(childLink, parentJoint, toChild, fromParent) {
  this->upperLimit = upperLimit;
  this->lowerLimit = lowerLimit;
}

void RevoluteKJoint::setQpos(const std::vector<PxReal> &q) {
  qpos = q[0];
  if (q[0] > upperLimit || q[0] < lowerLimit) {
    std::cerr << "Trying to set revelute joint angle out of the limit" << std::endl;
  }
  PxTransform targetPose = passThrough(PxTransform({qpos, {1, 0, 0}}));
  childLink->setGlobalPose(targetPose);
}
void RevoluteKJoint::driveQpos(const std::vector<PxReal> &qpos) {
  PxReal targetQpos = qpos[0];
  targetQpos = targetQpos > upperLimit ? upperLimit : targetQpos;
  targetQpos = targetQpos < lowerLimit ? lowerLimit : targetQpos;
  PxTransform targetPose = passThrough(PxTransform({targetQpos, {1, 0, 0}}));
  childLink->setKinematicTarget(targetPose);
}
std::vector<std::tuple<PxReal, PxReal>> SingleDOFKJoint::getLimits() const {
  std::tuple<PxReal, PxReal> limits = {upperLimit, lowerLimit};
  return {limits};
}

// Continuous KJoint
ContinuousKJoint::ContinuousKJoint(PxRigidDynamic *childLink, KJoint *parentJoint,
                                   const PxTransform &toChild, const PxTransform &fromParent)
    : SingleDOFKJoint(childLink, parentJoint, toChild, fromParent) {
  upperLimit = std::numeric_limits<PxReal>::max();
  lowerLimit = std::numeric_limits<PxReal>::min();
}
void ContinuousKJoint::setQpos(const std::vector<PxReal> &q) {
  qpos = q[0];
  PxTransform targetPose = passThrough(PxTransform({qpos, {1, 0, 0}}));
  childLink->setGlobalPose(targetPose);
}
void ContinuousKJoint::driveQpos(const std::vector<PxReal> &qpos) {
  PxReal targetQpos = qpos[0];
  PxTransform targetPose = passThrough(PxTransform({targetQpos, {1, 0, 0}}));
  childLink->setKinematicTarget(targetPose);
}

// Prismatic Joint
PrismaticKJoint::PrismaticKJoint(PxRigidDynamic *childLink, KJoint *parentJoint,
                                 const PxTransform &toChild, const PxTransform &fromParent,
                                 PxReal upperLimit, PxReal lowerLimit)
    : SingleDOFKJoint(childLink, parentJoint, toChild, fromParent) {
  this->upperLimit = upperLimit;
  this->lowerLimit = lowerLimit;
}
void PrismaticKJoint::setQpos(const std::vector<PxReal> &q) {
  qpos = q[0];
  if (q[0] > upperLimit || q[0] < lowerLimit) {
    std::cerr << "Trying to set revelute joint angle out of the limit" << std::endl;
  }
  PxTransform targetPose = passThrough(PxTransform(PxVec3(qpos, 0, 0)));
  childLink->setGlobalPose(targetPose);
}
void PrismaticKJoint::driveQpos(const std::vector<PxReal> &qpos) {
  PxReal targetQpos = qpos[0];
  targetQpos = targetQpos > upperLimit ? upperLimit : targetQpos;
  targetQpos = targetQpos > lowerLimit ? lowerLimit : targetQpos;
  PxTransform targetPose = passThrough(PxTransform(PxVec3(targetQpos, 0, 0)));
  childLink->setKinematicTarget(targetPose);
}

// Fixed Joint
FixedKJoint::FixedKJoint(PxRigidDynamic *childLink, KJoint *parentJoint,
                         const PxTransform &toChild, const PxTransform &fromParent)
    : KJoint(childLink, parentJoint, toChild, fromParent) {}
void FixedKJoint::setQpos(const std::vector<PxReal> &qpos) {
  assert(qpos.size() == 0);
  PxTransform targetPose = passThrough(PxTransform(PxIdentity));
  childLink->setGlobalPose(targetPose);
}
std::vector<std::tuple<PxReal, PxReal>> FixedKJoint::getLimits() const { return {}; }
void FixedKJoint::driveQpos(const std::vector<PxReal> &qpos) {
  if (!qpos.empty()) {
    throw std::runtime_error("Fixed joint can not be drove by real qpos");
  }
  PxTransform targetPose = passThrough(PxTransform(PxIdentity));
  childLink->setKinematicTarget(targetPose);
}
