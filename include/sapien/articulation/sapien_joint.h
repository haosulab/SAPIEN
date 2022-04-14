#pragma once
#include <PxPhysicsAPI.h>
#include <array>
#include <string>
#include <vector>

namespace sapien {
using namespace physx;

class SArticulationBase;
class SArticulation;
class SKArticulation;
class SLinkBase;
class SLink;

class SJointBase {
protected:
  std::string mName;
  SLinkBase *mParentLink;
  SLinkBase *mChildLink;

public:
  inline std::string getName() const { return mName; }
  inline void setName(std::string const &name) { mName = name; }

  inline SLinkBase *getParentLink() { return mParentLink; }
  inline SLinkBase *getChildLink() { return mChildLink; }

  virtual uint32_t getDof() const = 0;

  virtual std::vector<std::array<physx::PxReal, 2>> getLimits() = 0;
  virtual void setLimits(std::vector<std::array<physx::PxReal, 2>> const &limits) = 0;

  /* Get the pose of this joint in parent frame */
  virtual PxTransform getParentPose() const = 0;

  /* Get the pose of this joint in child frame */
  virtual PxTransform getChildPose() const = 0;

  virtual PxArticulationJointType::Enum getType() const = 0;

  virtual SArticulationBase *getArticulation() const = 0;

  virtual ~SJointBase() = default;

protected:
  SJointBase(SLinkBase *parent, SLinkBase *child);
};

class SJoint : public SJointBase {
  friend class LinkBuilder;
  SArticulation *mArticulation;
  PxArticulationJointReducedCoordinate *mPxJoint;

public:
  PxArticulationJointReducedCoordinate *getPxJoint();

  uint32_t getDof() const override;

  std::vector<std::array<physx::PxReal, 2>> getLimits() override;
  void setLimits(std::vector<std::array<physx::PxReal, 2>> const &limits) override;
  std::vector<PxArticulationAxis::Enum> getAxes() const;

  void setFriction(PxReal coef);
  void setDriveProperty(PxReal stiffness, PxReal damping, PxReal forceLimit = PX_MAX_F32,
                        bool useAcceleration = false);

  PxReal getFriction() const;
  PxReal getDriveStiffness() const;
  PxReal getDriveDamping() const;
  PxReal getDriveForceLimit() const;
  PxReal getDriveTarget() const;
  PxReal getDriveVelocityTarget() const;
  bool getDriveIsAcceleration() const;

  void setDriveVelocityTarget(std::vector<PxReal> const &v);
  void setDriveVelocityTarget(PxReal v);

  void setDriveTarget(std::vector<PxReal> const &p);
  void setDriveTarget(PxReal p);

  PxTransform getGlobalPose() const;

  virtual PxTransform getParentPose() const override;
  virtual PxTransform getChildPose() const override;

  virtual PxArticulationJointType::Enum getType() const override;

  SArticulationBase *getArticulation() const override;

private:
  SJoint(SArticulation *articulation, SLink *parent, SLink *child,
         PxArticulationJointReducedCoordinate *pxJoint);
};

} // namespace sapien
