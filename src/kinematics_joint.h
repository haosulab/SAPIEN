//
// Created by sim on 9/26/19.
//

#pragma once

#include <PxPhysicsAPI.h>
#include <map>
#include <string>
#include <tuple>
#include <vector>

namespace sapien {
using namespace physx;

enum JointType { UNDEFINED = 0, REVOLUTE = 1, CONTINUOUS = 2, FIXED = 3, PRISMATIC = 4 };

class KJoint {
public:
  virtual ~KJoint() {}
  std::string name;
  PxRigidDynamic *childLink;
  PxRigidDynamic *parentLink;

protected:
  KJoint *parent;
  PxTransform poseFromParent;
  PxTransform poseToChild;
  JointType type = UNDEFINED;

public:
  KJoint(PxRigidDynamic *childLink, KJoint *parentJoint, const PxTransform &toChild,
         const PxTransform &fromParent);
  PxTransform passThroughKinematicsDrive(const PxTransform &T);
  PxTransform passThroughGlobalPose(const PxTransform &T);
  virtual std::vector<std::tuple<PxReal, PxReal>> getLimits() const = 0;
  virtual uint32_t getDof() const = 0;
  virtual PxReal getQpos() const = 0;
  virtual void setQpos(const std::vector<PxReal> &qpos) = 0;
  virtual void driveQpos(const std::vector<PxReal> &qpos) = 0;
  std::vector<KJoint *> children;
  void setName(const std::string &n) { name = n; }
};

class SingleDOFKJoint : public KJoint {
protected:
  PxReal qvel = 0;
  PxReal qpos = 0;
  PxReal upperLimit;
  PxReal lowerLimit;

public:
  SingleDOFKJoint(PxRigidDynamic *childLink, KJoint *parentJoint, const PxTransform &toChild,
                  const PxTransform &fromParent)
      : KJoint(childLink, parentJoint, toChild, fromParent){};

  uint32_t getDof() const override { return 1; }
  std::vector<std::tuple<PxReal, PxReal>> getLimits() const override;
  PxReal getQpos() const override { return qpos; }
};

class RevoluteKJoint : public SingleDOFKJoint {
public:
  RevoluteKJoint(PxRigidDynamic *childLink, KJoint *parentJoint, const PxTransform &toChild,
                 const PxTransform &fromParent, PxReal upperLimit, PxReal lowerLimit);

  void driveQpos(const std::vector<PxReal> &qpos) override;
  void setQpos(const std::vector<PxReal> &qpos) override;
};

class ContinuousKJoint : public SingleDOFKJoint {
public:
  ContinuousKJoint(PxRigidDynamic *childLink, KJoint *parentJoint, const PxTransform &toChild,
                   const PxTransform &fromParent);

  void driveQpos(const std::vector<PxReal> &qpos) override;
  void setQpos(const std::vector<PxReal> &qpos) override;
};
class PrismaticKJoint : public SingleDOFKJoint {
public:
  PrismaticKJoint(PxRigidDynamic *childLink, KJoint *parentJoint, const PxTransform &toChild,
                  const PxTransform &fromParent, PxReal upperLimit, PxReal lowerLimit);

  void driveQpos(const std::vector<PxReal> &qpos) override;

  void setQpos(const std::vector<PxReal> &qpos) override;
};

class FixedKJoint : public KJoint {
public:
  FixedKJoint(PxRigidDynamic *childLink, KJoint *parentJoint, const PxTransform &toChild,
              const PxTransform &fromParent);
  ;

  uint32_t getDof() const override { return 0; }
  void setQpos(const std::vector<PxReal> &qpos) override;
  void driveQpos(const std::vector<PxReal> &qpos) override;
  std::vector<std::tuple<PxReal, PxReal>> getLimits() const override;
  PxReal getQpos() const override { return 0; }
};

}
