#pragma once
#include "id_generator.h"
#include "render_interface.h"
#include "simulation.h"
#include <PxPhysicsAPI.h>
#include <cmath>
#include <memory>
#include <vector>

using namespace physx;

enum JointType { UNDEFINED = 0, REVOLUTE = 1, CONTINUOUS = 2, FIXED = 3, PRISMATIC = 4 };

struct KJoint {
  PxRigidDynamic *childLink;
  PxRigidDynamic *parentLink;
  PxTransform poseToChild;
  PxTransform poseFromParent;
  std::vector<KJoint *> children;
  KJoint *parent;
  JointType type;
  KJoint(PxRigidDynamic *cLink, KJoint *pJoint, const PxTransform &toChild, const PxTransform &fromParent) {
    childLink = cLink;
    parentLink = pJoint ? pJoint->childLink : nullptr;
    poseToChild = toChild;
    poseFromParent = fromParent;
    if (pJoint) {
      pJoint->children.push_back(this);
    }
    parent = pJoint;
  }

  void passThrough(const PxTransform &T) {
    PxTransform parentPose(PxIdentity);
    if (parentLink) {
      parentPose = parentLink->getGlobalPose();
      parentLink->getKinematicTarget(parentPose);
    }
    PxTransform toSet = parentPose * poseFromParent * T * poseToChild;
    childLink->setKinematicTarget(toSet);
  }

  virtual void update() {
    for (KJoint *joint : children) {
      joint->update();
    }
  };

  virtual void advance(PxReal dt) {
    for (KJoint *joint : children) {
      joint->advance(dt);
    }
  }
};

struct KRevoluteJoint : KJoint {
  PxReal velocity = 0;
  PxReal upperLimit;
  PxReal lowerLimit;
  PxReal angle = 0;
  KRevoluteJoint(PxRigidDynamic *cLink, KJoint *pJoint, const PxTransform &toChild,
                 const PxTransform &fromParent, PxReal ul, PxReal ll)
      : KJoint(cLink, pJoint, toChild, fromParent) {
    type = REVOLUTE;
    upperLimit = ul;
    lowerLimit = ll;
  }

  void update() {
    passThrough(PxTransform({angle, {1, 0, 0}}));
    KJoint::update();
  }

  void advance(PxReal dt) {
    angle += velocity * dt;
    angle = angle > upperLimit ? upperLimit : angle;
    angle = angle < lowerLimit ? lowerLimit : angle;
    KJoint::advance(dt);
  }
};

struct KContinuousJoint : KJoint {
  PxReal velocity = 0;
  PxReal angle = 0;
  KContinuousJoint(PxRigidDynamic *cLink, KJoint *pJoint, const PxTransform &toChild,
                   const PxTransform &fromParent)
      : KJoint(cLink, pJoint, toChild, fromParent) {
    type = CONTINUOUS;
  }

  void update() {
    passThrough(PxTransform({angle, {1, 0, 0}}));
    KJoint::update();
  }

  void advance(PxReal dt) {
    angle += velocity * dt;
    angle = fmod(angle, PxPi);
    KJoint::advance(dt);
  }
};

struct KFixedJoint : KJoint {
  KFixedJoint(PxRigidDynamic *cLink, KJoint *pJoint, const PxTransform &toChild, const PxTransform &fromParent)
      : KJoint(cLink, pJoint, toChild, fromParent) {
    type = FIXED;
  }

  void update() {
    passThrough(PxTransform(PxIdentity));
    KJoint::update();
  }

  void advance(PxReal dt) { KJoint::advance(dt); }
};

struct KPrismaticJoint : KJoint {
  PxReal velocity = 0;
  PxReal upperLimit;
  PxReal lowerLimit;
  PxReal translation = 0;
  KPrismaticJoint(PxRigidDynamic *cLink, KJoint *pJoint, const PxTransform &toChild,
                  const PxTransform &fromParent, PxReal ul, PxReal ll)
      : KJoint(cLink, pJoint, toChild, fromParent) {
    type = PRISMATIC;
    upperLimit = ul;
    lowerLimit = ll;
  }
  void update() {
    passThrough(PxTransform(PxVec3(translation, 0, 0)));
    KJoint::update();
  }
  void advance(PxReal dt) {
    translation += velocity * dt;
    translation = translation > upperLimit ? upperLimit : translation;
    translation = translation < lowerLimit ? lowerLimit : translation;
    KJoint::advance(dt);
  }
};

class KinematicArticulation {
  KJoint *root;
  std::vector<KJoint *> joints;
  std::map<std::string, KJoint *> namedJoints;

  PxSimulation *mSimulation = nullptr;
  PxPhysics *mPhysicsSDK = nullptr;
  PxCooking *mCooking = nullptr;
  Renderer::IPhysxRenderer *mRenderer = nullptr;

public:
  KinematicArticulation(PxSimulation *simulation);
  ~KinematicArticulation();

  KJoint *createRevoluteJoint(KJoint *parent, PxRigidDynamic *link,
                              const PxTransform &poseFromParent, const PxTransform &poseFromChild,
                              PxReal upperLimit, PxReal lowerLimit, const std::string &name = "");

  KJoint *createContinuousJoint(KJoint *parent, PxRigidDynamic *link,
                                const PxTransform &poseFromParent,
                                const PxTransform &poseFromChild, const std::string &name = "");

  KJoint *createFixedJoint(KJoint *parent, PxRigidDynamic *link, const PxTransform &poseFromParent,
                           const PxTransform &poseFromChild, const std::string &name = "");

  KJoint *createPrismaticJoint(KJoint *parent, PxRigidDynamic *link,
                               const PxTransform &poseFromParent, const PxTransform &poseFromChild,
                               PxReal upperLimit, PxReal lowerLimit, const std::string &name = "");

  void update();

  void setRootGlobalPose(const PxTransform &T);

  KJoint *getRoot() {return root;}

  void setAllPos(const std::vector<PxReal> &v_pos);

  void setAllVelocity(const std::vector<PxReal> &v_vel);

  KJoint *getJoint(PxU32 index);

  KJoint *getJoint(const std::string &name);

  PxU32 getNbJoint();
};
