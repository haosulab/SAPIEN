#pragma once
#include "sapien_joint.h"

namespace sapien {
class SKLink;
class SKArticulation;

class SKJoint : public SJointBase {
  friend class LinkBuilder;

  SKArticulation *mArticulatoin;

  PxTransform joint2parent;
  PxTransform child2joint;

public:
  virtual std::vector<PxReal> getPos() const = 0;
  virtual std::vector<PxReal> getVel() const = 0;
  virtual void setPos(std::vector<PxReal> const &v) = 0;
  virtual void setVel(std::vector<PxReal> const &v) = 0;
  SKJoint(SKArticulation *articulation, SKLink *parent, SKLink *child);

  void setParentPose(PxTransform const &pose) { joint2parent = pose; }
  void setChildPose(PxTransform const &pose) { child2joint = pose.getInverse(); }

  virtual PxTransform getJointPose() const = 0;
  inline PxTransform getChild2ParentTransform() const {
    return joint2parent * getJointPose() * child2joint;
  }
};

class SKJointSingleDof : public SKJoint {
protected:
  PxReal vel = 0;
  PxReal pos = 0;
  PxReal lowerLimit;
  PxReal upperLimit;

public:
  SKJointSingleDof();

  inline uint32_t getDof() const override { return 1; }
  inline std::vector<PxReal> getPos() const override { return {pos}; }
  inline std::vector<PxReal> getVel() const override { return {vel}; }
  void setPos(std::vector<PxReal> const &v) override;
  void setVel(std::vector<PxReal> const &v) override;
  inline std::vector<std::array<PxReal, 2>> getLimits() override {
    return {{lowerLimit, upperLimit}};
  };
  void setLimits(std::vector<std::array<PxReal, 2>> const &limits) override;
  using SKJoint::SKJoint;
};

class SKJointRevolute : public SKJointSingleDof {
public:
  using SKJointSingleDof::SKJointSingleDof;

  PxTransform getJointPose() const override;
};

class SKJointPrismatic : public SKJointSingleDof {
public:
  using SKJointSingleDof::SKJointSingleDof;

  PxTransform getJointPose() const override;
};

class SKJointFixed : public SKJoint {
public:
  inline uint32_t getDof() const override { return 0; }
  inline std::vector<PxReal> getPos() const override { return {}; }
  inline std::vector<PxReal> getVel() const override { return {}; }
  void setPos(std::vector<PxReal> const &v) override;
  void setVel(std::vector<PxReal> const &v) override;
  inline std::vector<std::array<PxReal, 2>> getLimits() override { return {}; }
  void setLimits(std::vector<std::array<PxReal, 2>> const &limits) override;

  inline PxTransform getJointPose() const override { return {{0, 0, 0}, PxIdentity}; }

public:
  using SKJoint::SKJoint;
};

} // namespace sapien
