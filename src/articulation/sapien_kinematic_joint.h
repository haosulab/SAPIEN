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
  virtual void setDriveProperties(PxReal accStiffness, PxReal accDamping, PxReal maxVel) = 0;
  virtual void setDriveTarget(std::vector<PxReal> const &p) = 0;
  virtual void setDriveVelocityTarget(std::vector<PxReal> const &v) = 0;
  SKJoint(SKArticulation *articulation, SKLink *parent, SKLink *child);

  void setParentPose(PxTransform const &pose) { joint2parent = pose; }
  void setChildPose(PxTransform const &pose) { child2joint = pose.getInverse(); }

  virtual inline PxTransform getParentPose() const override { return joint2parent; }

  virtual inline PxTransform getChildPose() const override { return child2joint.getInverse(); }

  virtual PxTransform getJointPose() const = 0;
  inline PxTransform getChild2ParentTransform() const {
    return joint2parent * getJointPose() * child2joint;
  }

  // update pos by vel
  virtual void updatePos(PxReal dt) = 0;
  SArticulationBase *getArticulation() const override;

public:
  using SJointBase::SJointBase;
  SKJoint(SKJoint const &) = delete;
  SKJoint &operator=(SKJoint const &) = delete;
  ~SKJoint() = default;
};

class SKJointSingleDof : public SKJoint {
protected:
  PxReal vel = 0;
  PxReal pos = 0;
  PxReal lowerLimit;
  PxReal upperLimit;

  PxReal targetPos = 0;
  PxReal targetVel = 0;

  PxReal stiffness = 0;
  PxReal damping = 0;
  PxReal maxVel = PX_MAX_F32;

  PxReal acc = 0;

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

  void setDriveProperties(PxReal accStiffness, PxReal accDamping, PxReal maxVel) override;
  void setDriveTarget(std::vector<PxReal> const &p) override;
  void setDriveVelocityTarget(std::vector<PxReal> const &v) override;
  virtual void updatePos(PxReal dt) override;

  virtual inline PxArticulationJointType::Enum getType() const override {
    return PxArticulationJointType::eUNDEFINED;
  };

  using SKJoint::SKJoint;
};

class SKJointRevolute : public SKJointSingleDof {
public:
  using SKJointSingleDof::SKJointSingleDof;

  virtual inline PxArticulationJointType::Enum getType() const override {
    return PxArticulationJointType::eREVOLUTE;
  };

  PxTransform getJointPose() const override;
};

class SKJointPrismatic : public SKJointSingleDof {
public:
  using SKJointSingleDof::SKJointSingleDof;
  
  virtual inline PxArticulationJointType::Enum getType() const override {
    return PxArticulationJointType::ePRISMATIC;
  };

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

  inline void setDriveProperties(PxReal accStiffness, PxReal accDamping, PxReal maxVel) override {}
  inline void setDriveTarget(std::vector<PxReal> const &p) override {}
  inline void setDriveVelocityTarget(std::vector<PxReal> const &v) override {}
  inline void updatePos(PxReal dt) override{};

  inline PxTransform getJointPose() const override { return {{0, 0, 0}, PxIdentity}; }

  virtual inline PxArticulationJointType::Enum getType() const override {
    return PxArticulationJointType::eFIX;
  };

public:
  using SKJoint::SKJoint;
};

} // namespace sapien
