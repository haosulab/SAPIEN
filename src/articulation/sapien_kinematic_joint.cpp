#include "sapien_kinematic_joint.h"
#include "sapien_link.h"
#include <spdlog/spdlog.h>

namespace sapien {
SKJoint::SKJoint(SKArticulation *articulation, SKLink *parent, SKLink *child)
    : SJointBase(parent, child), mArticulatoin(articulation) {}

void SKJointSingleDof::setLimits(const std::vector<std::array<PxReal, 2>> &limits) {
  if (limits.size() != 1) {
    spdlog::error("setLimits failed: argument does not match joint DOF");
  }
  lowerLimit = limits[0][0];
  upperLimit = limits[0][1];
}

void SKJointSingleDof::setPos(const std::vector<PxReal> &v) {
  if (v.size() != 1) {
    spdlog::error("setPos failed: argument does not match joint DOF");
  }
  pos = v[0];
  if (pos < lowerLimit) {
    pos = lowerLimit;
  } else if (pos > upperLimit) {
    pos = upperLimit;
  }
}

void SKJointSingleDof::setVel(const std::vector<PxReal> &v) {
  if (v.size() != 1) {
    spdlog::error("setPos failed: argument does not match joint DOF");
  }
  vel = v[0];
}

void SKJointSingleDof::setDriveProperties(PxReal accStiffness, PxReal accDamping, PxReal vmax) {
  stiffness = accStiffness;
  damping = accDamping;
  maxVel = vmax;
}

void SKJointSingleDof::setDriveTarget(std::vector<PxReal> const &p) {
  if (p.size() != 1) {
    spdlog::error("setDriveTarget failed: argument does not match joint DOF");
  }
  targetPos = p[0];
}
void SKJointSingleDof::setDriveVelocityTarget(std::vector<PxReal> const &v) {
  if (v.size() != 1) {
    spdlog::error("setDriveVelocityTarget failed: argument does not match joint DOF");
  }
  targetVel = v[0];
}

void SKJointSingleDof::updatePos(PxReal dt) {
  acc = stiffness * (targetPos - pos) + damping * (targetVel - vel);
  vel += acc * dt;
  if (vel > maxVel) {
    vel = maxVel;
  } else if (vel < -maxVel) {
    vel = -maxVel;
  }

  pos += vel * dt;
  pos = std::clamp(pos, lowerLimit, upperLimit);
}

void SKJointFixed::setLimits(const std::vector<std::array<physx::PxReal, 2>> &limits) {
  if (limits.size()) {
    spdlog::error("setLimits failed: fixed joint does not support limits");
  }
}

void SKJointFixed::setPos(const std::vector<PxReal> &v) {
  if (v.size() != 0) {
    spdlog::error("setPos failed: fixed joint does not support joint pos");
  }
}

void SKJointFixed::setVel(const std::vector<PxReal> &v) {
  if (v.size() != 0) {
    spdlog::error("setVel failed: fixed joint does not support joint vel");
  }
}

PxTransform SKJointRevolute::getJointPose() const {
  return PxTransform({{0, 0, 0}, PxQuat(pos, {1, 0, 0})});
}

PxTransform SKJointPrismatic::getJointPose() const {
  return PxTransform({{pos, 0, 0}, PxIdentity});
}

} // namespace sapien
