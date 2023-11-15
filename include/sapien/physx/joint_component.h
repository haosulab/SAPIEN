#pragma once
#include "base_component.h"
#include "rigid_component.h"
#include "sapien/math/conversion.h"

namespace sapien {
class Entity;
namespace physx {

class PhysxJointComponent : public PhysxBaseComponent {
public:
  PhysxJointComponent(std::shared_ptr<PhysxRigidBodyComponent> body);

  /** should be called internally when PxActor changes for parent or child */
  virtual void internalRefresh();

  void onAddToScene(Scene &scene) override;
  void onRemoveFromScene(Scene &scene) override;

  void setParent(std::shared_ptr<PhysxRigidBaseComponent> body);
  std::shared_ptr<PhysxRigidBaseComponent> getParent() const;

  void setParentAnchorPose(Pose const &pose);
  Pose getParentAnchorPose() const;
  void setChildAnchorPose(Pose const &pose);
  Pose getChildAnchorPose() const;

  Pose getRelativePose() const;

  // TODO: serialize
  void setInvMassScales(float, float);
  void setInvInertiaScales(float, float);

  virtual ::physx::PxJoint *getPxJoint() const = 0;
  ~PhysxJointComponent();

protected:
  std::shared_ptr<PhysxRigidBodyComponent> mChild;
  std::shared_ptr<PhysxRigidBaseComponent> mParent;
};

class PhysxDriveComponent : public PhysxJointComponent {
public:
  enum class DriveMode { eFORCE, eACCELERATION };

  static std::shared_ptr<PhysxDriveComponent>
  Create(std::shared_ptr<PhysxRigidBodyComponent> body);

  /** should only be called internally */
  PhysxDriveComponent(std::shared_ptr<PhysxRigidBodyComponent> body);

  void setXLimit(float low, float high, float stiffness, float damping);
  void setYLimit(float low, float high, float stiffness, float damping);
  void setZLimit(float low, float high, float stiffness, float damping);
  void setXTwistLimit(float low, float high, float stiffness, float damping);
  void setYZConeLimit(float yAngle, float zAngle, float stiffness, float damping);
  void setYZPyramidLimit(float yLow, float yHigh, float zLow, float zHigh, float stiffness,
                         float damping);

  std::tuple<float, float, float, float> getXLimit() const;
  std::tuple<float, float, float, float> getYLimit() const;
  std::tuple<float, float, float, float> getZLimit() const;
  std::tuple<float, float, float, float> getXTwistLimit() const;
  std::tuple<float, float, float, float> getYZConeLimit() const;
  std::tuple<float, float, float, float, float, float> getZPyramidLimit() const;

  void setXDriveProperties(float stiffness, float damping, float forceLimit, DriveMode mode);
  void setYDriveProperties(float stiffness, float damping, float forceLimit, DriveMode mode);
  void setZDriveProperties(float stiffness, float damping, float forceLimit, DriveMode mode);
  void setXTwistDriveProperties(float stiffness, float damping, float forceLimit, DriveMode mode);
  void setYZSwingDriveProperties(float stiffness, float damping, float forceLimit, DriveMode mode);
  void setSlerpDriveProperties(float stiffness, float damping, float forceLimit, DriveMode mode);

  std::tuple<float, float, float, DriveMode> getXDriveProperties() const;
  std::tuple<float, float, float, DriveMode> getYDriveProperties() const;
  std::tuple<float, float, float, DriveMode> getZDriveProperties() const;
  std::tuple<float, float, float, DriveMode> getXTwistDriveProperties() const;
  std::tuple<float, float, float, DriveMode> getYZSwingDriveProperties() const;
  std::tuple<float, float, float, DriveMode> getSlerpDriveProperties() const;

  void setDriveTarget(Pose const &pose);
  Pose getDriveTarget() const;

  void setDriveTargetVelocity(Vec3 const &linear, Vec3 const &angular);
  std::tuple<Vec3, Vec3> getDriveTargetVelocity() const;

  ::physx::PxJoint *getPxJoint() const override { return mJoint; }

  template <class Archive> void save(Archive &ar) const;
  template <class Archive>
  static void load_and_construct(Archive &ar, cereal::construct<PhysxDriveComponent> &construct);

  ~PhysxDriveComponent();

private:
  void setLinearLimit(::physx::PxD6Axis::Enum axis, float low, float high, float stiffness,
                      float damping);
  void setDrive(::physx::PxD6Drive::Enum drive, float stiffness, float damping, float forceLimit,
                DriveMode mode);

  ::physx::PxD6Joint *mJoint;
};

class PhysxGearComponent : public PhysxJointComponent {

public:
  static std::shared_ptr<PhysxGearComponent> Create(std::shared_ptr<PhysxRigidBodyComponent> body);

  /** should only be called internally */
  PhysxGearComponent(std::shared_ptr<PhysxRigidBodyComponent> body);

  float getGearRatio() const;
  void setGearRatio(float ratio);

  void enableHinges();
  bool getHingesEnabled() const { return mHingesEnabled; };

  ::physx::PxJoint *getPxJoint() const override { return mJoint; }

  void internalRefresh() override;

  ~PhysxGearComponent();

private:
  bool mHingesEnabled{false};
  ::physx::PxGearJoint *mJoint;
};

class PhysxDistanceJointComponent : public PhysxJointComponent {
public:
  static std::shared_ptr<PhysxDistanceJointComponent>
  Create(std::shared_ptr<PhysxRigidBodyComponent> body);
  PhysxDistanceJointComponent(std::shared_ptr<PhysxRigidBodyComponent> body);

  void setLimit(float low, float high, float stiffness, float damping);
  float getStiffness() const;
  float getDamping() const;
  Eigen::Vector2f getLimit() const;

  float getDistance() const;

  ::physx::PxJoint *getPxJoint() const override { return mJoint; }

  ~PhysxDistanceJointComponent();

private:
  ::physx::PxDistanceJoint *mJoint;
};

template <class Archive> void PhysxDriveComponent::save(Archive &ar) const {
  ar(mChild, mParent);
  ar(PxTransformToPose(mJoint->getLocalPose(::physx::PxJointActorIndex::eACTOR0)),
     PxTransformToPose(mJoint->getLocalPose(::physx::PxJointActorIndex::eACTOR1)));

  for (auto axis :
       {::physx::PxD6Axis::eX, ::physx::PxD6Axis::eY, ::physx::PxD6Axis::eZ,
        ::physx::PxD6Axis::eTWIST, ::physx::PxD6Axis::eSWING1, ::physx::PxD6Axis::eSWING2}) {
    int motion = mJoint->getMotion(axis);
    ar(motion);
  }

  // linear limit
  for (auto axis : {::physx::PxD6Axis::eX, ::physx::PxD6Axis::eY, ::physx::PxD6Axis::eZ}) {
    auto limit = mJoint->getLinearLimit(axis);
    ar(limit.lower, limit.upper, limit.restitution, limit.bounceThreshold, limit.stiffness,
       limit.damping);
  }

  // twist limit
  {
    auto limit = mJoint->getTwistLimit();
    ar(limit.lower, limit.upper, limit.restitution, limit.bounceThreshold, limit.stiffness,
       limit.damping);
  }

  // swing cone limit
  {
    auto limit = mJoint->getSwingLimit();
    ar(limit.yAngle, limit.zAngle, limit.restitution, limit.bounceThreshold, limit.stiffness,
       limit.damping);
  }

  // swing pyramid limit
  {
    auto limit = mJoint->getPyramidSwingLimit();
    ar(limit.yAngleMin, limit.yAngleMax, limit.zAngleMin, limit.zAngleMax, limit.restitution,
       limit.bounceThreshold, limit.stiffness, limit.damping);
  }

  // drive
  for (auto axis :
       {::physx::PxD6Drive::eX, ::physx::PxD6Drive::eY, ::physx::PxD6Drive::eZ,
        ::physx::PxD6Drive::eTWIST, ::physx::PxD6Drive::eSWING, ::physx::PxD6Drive::eSLERP}) {
    auto drive = mJoint->getDrive(axis);
    uint32_t flags = drive.flags;
    ar(drive.stiffness, drive.damping, drive.forceLimit, flags);
  }

  // target
  {
    auto pose = mJoint->getDrivePosition();
    ::physx::PxVec3 linear, angular;
    mJoint->getDriveVelocity(linear, angular);
    ar(PxTransformToPose(pose), PxVec3ToVec3(linear), PxVec3ToVec3(angular));
  }

  ar(cereal::base_class<PhysxBaseComponent>(this));
}

template <class Archive>
void PhysxDriveComponent::load_and_construct(Archive &ar,
                                             cereal::construct<PhysxDriveComponent> &construct) {

  std::shared_ptr<PhysxRigidBodyComponent> child;
  std::shared_ptr<PhysxRigidBaseComponent> parent;
  ar(child, parent);

  construct(child);
  construct->setParent(parent);

  Pose p0, p1;
  ar(p0, p1);
  construct->mJoint->setLocalPose(::physx::PxJointActorIndex::eACTOR0, PoseToPxTransform(p0));
  construct->mJoint->setLocalPose(::physx::PxJointActorIndex::eACTOR1, PoseToPxTransform(p1));

  for (auto axis :
       {::physx::PxD6Axis::eX, ::physx::PxD6Axis::eY, ::physx::PxD6Axis::eZ,
        ::physx::PxD6Axis::eTWIST, ::physx::PxD6Axis::eSWING1, ::physx::PxD6Axis::eSWING2}) {
    int motion;
    ar(motion);
    construct->mJoint->setMotion(axis, static_cast<::physx::PxD6Motion::Enum>(motion));
  }

  // linear limit
  for (auto axis : {::physx::PxD6Axis::eX, ::physx::PxD6Axis::eY, ::physx::PxD6Axis::eZ}) {
    ::physx::PxJointLinearLimitPair limit(0.f, 0.f, {0.f, 0.f});
    ar(limit.lower, limit.upper, limit.restitution, limit.bounceThreshold, limit.stiffness,
       limit.damping);
    construct->mJoint->setLinearLimit(axis, limit);
  }

  // twist limit
  {
    ::physx::PxJointAngularLimitPair limit(0.f, 0.f, {0.f, 0.f});
    ar(limit.lower, limit.upper, limit.restitution, limit.bounceThreshold, limit.stiffness,
       limit.damping);
    construct->mJoint->setTwistLimit(limit);
  }

  // swing cone limit
  {
    ::physx::PxJointLimitCone limit(0, 0);
    ar(limit.yAngle, limit.zAngle, limit.restitution, limit.bounceThreshold, limit.stiffness,
       limit.damping);
    construct->mJoint->setSwingLimit(limit);
  }

  // swing pyramid limit
  {
    ::physx::PxJointLimitPyramid limit(0, 0, 0, 0);
    ar(limit.yAngleMin, limit.yAngleMax, limit.zAngleMin, limit.zAngleMax, limit.restitution,
       limit.bounceThreshold, limit.stiffness, limit.damping);
    construct->mJoint->setPyramidSwingLimit(limit);
  }

  // drive
  for (auto axis :
       {::physx::PxD6Drive::eX, ::physx::PxD6Drive::eY, ::physx::PxD6Drive::eZ,
        ::physx::PxD6Drive::eTWIST, ::physx::PxD6Drive::eSWING, ::physx::PxD6Drive::eSLERP}) {
    uint32_t flags;
    ::physx::PxD6JointDrive drive;
    ar(drive.stiffness, drive.damping, drive.forceLimit, flags);
    drive.flags = static_cast<::physx::PxD6JointDriveFlags>(flags);
    construct->mJoint->setDrive(axis, drive);
  }

  // target
  {
    Pose pose;
    Vec3 v, w;
    ar(pose, v, w);
    construct->mJoint->setDrivePosition(PoseToPxTransform(pose));
    construct->mJoint->setDriveVelocity(Vec3ToPxVec3(v), Vec3ToPxVec3(w));
  }

  ar(cereal::base_class<PhysxBaseComponent>(construct.ptr()));
}

} // namespace physx
} // namespace sapien

CEREAL_REGISTER_TYPE(sapien::physx::PhysxDriveComponent);
CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::physx::PhysxRigidBodyComponent,
                                     sapien::physx::PhysxBaseComponent);
