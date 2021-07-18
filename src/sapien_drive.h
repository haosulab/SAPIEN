#pragma once
#include <PxPhysicsAPI.h>
#include <tuple>

namespace sapien {
class SScene;
class SActorBase;
using namespace physx;

class SDrive {
  friend SScene;

protected:
  SScene *mScene;
  SActorBase *mActor1;
  SActorBase *mActor2;

public:
  SDrive(SScene *scene, SActorBase *actor1, SActorBase *actor2);

  inline SActorBase *getActor1() const { return mActor1; };
  inline SActorBase *getActor2() const { return mActor2; };

  virtual PxJoint *getPxJoint() const = 0;

  PxTransform getLocalPose1() const;
  PxTransform getLocalPose2() const;

  virtual ~SDrive() = default;
};

class SDrive6D : public SDrive {
  friend SScene;

private:
  PxD6Joint *mJoint;

  SDrive6D(SScene *scene, SActorBase *actor1, PxTransform const &pose1, SActorBase *actor2,
           PxTransform const &pose2);

public:
  PxJoint *getPxJoint() const override;

  void setXLimit(float low, float high);
  void setYLimit(float low, float high);
  void setZLimit(float low, float high);
  void setXTwistLimit(float low, float high);
  void setYZConeLimit(float yangle, float zangle);
  void setYZPyramidLimit(float yangleMin, float yangleMax, float zangleMin, float zangleMax);
  void setDistanceLimit(float dist);

  void setXProperties(PxReal stiffness, PxReal damping, PxReal forceLimit, bool isAcceleration);
  void setYProperties(PxReal stiffness, PxReal damping, PxReal forceLimit, bool isAcceleration);
  void setZProperties(PxReal stiffness, PxReal damping, PxReal forceLimit, bool isAcceleration);

  void setXTwistDriveProperties(PxReal stiffness, PxReal damping, PxReal forceLimit,
                                bool isAcceleration);
  void setYZSwingDriveProperties(PxReal stiffness, PxReal damping, PxReal forceLimit,
                                 bool isAcceleration);

  void setSlerpProperties(PxReal stiffness, PxReal damping, PxReal forceLimit,
                          bool isAcceleration);

  void lockMotion(bool tx, bool ty, bool tz, bool rx, bool ry, bool rz);
  void freeMotion(bool tx, bool ty, bool tz, bool rx, bool ry, bool rz);

  void setTarget(PxTransform const &pose);
  PxTransform getTarget() const;

  void setTargetVelocity(PxVec3 const &velocity, PxVec3 const &angularVelocity);
  std::tuple<PxVec3, PxVec3> getTargetVelocity() const;
};

class SDriveRevolute : public SDrive {
  friend SScene;
  PxRevoluteJoint *mJoint;

public:
  SDriveRevolute(SScene *scene, SActorBase *actor1, PxTransform const &pose1, SActorBase *actor2,
                 PxTransform const &pose2);
  PxJoint *getPxJoint() const override;
  void setProperties(PxReal stiffness, PxReal damping, PxReal forceLimit, bool isAcceleration);

  void removeLimit();
  void setLimit(float low, float high);
  PxVec2 getLimit() const;

  void setLimitProperties(float stiffness, float damping);
  float getLimitStiffness() const;
  float getLimitDamping() const;
};

} // namespace sapien
