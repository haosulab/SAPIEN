#pragma once

#include "base_component.h"
#include "collision_shape.h"
#include "sapien/math/pose.h"
#include <PxPhysicsAPI.h>
#include <array>
#include <memory>
#include <string>

namespace sapien {
class Entity;
namespace physx {
class PhysxJointComponent;

class PhysxRigidBaseComponent : public PhysxBaseComponent {
public:
  using PhysxBaseComponent::PhysxBaseComponent;
  virtual std::shared_ptr<PhysxRigidBaseComponent>
  attachCollision(std::shared_ptr<PhysxCollisionShape> shape);

  std::vector<std::shared_ptr<PhysxCollisionShape>> getCollisionShapes() const;
  virtual ::physx::PxRigidActor *getPxActor() const = 0;

  AABB getGlobalAABBFast() const;
  AABB computeGlobalAABBTight() const;

  void syncPoseToEntity();

  void internalRegisterJoint(std::shared_ptr<PhysxJointComponent> joint);
  void internalUnregisterJoint(std::shared_ptr<PhysxJointComponent> joint);

  /** called when some drive is deleted */
  void internalClearExpiredJoints();

  /** returns true if the physx actor is added to a GPU-enabled scene */
  bool isUsingDirectGPUAPI() const;

protected:
  std::vector<std::weak_ptr<PhysxJointComponent>> mJoints;
  std::vector<std::shared_ptr<PhysxCollisionShape>> mCollisionShapes{};
};

class PhysxRigidStaticComponent : public PhysxRigidBaseComponent {
public:
  PhysxRigidStaticComponent();
  using PhysxRigidBaseComponent::PhysxRigidBaseComponent;
  ::physx::PxRigidStatic *getPxActor() const override { return mPxActor; };

  void onSetPose(Pose const &pose) override;
  void onAddToScene(Scene &scene) override;
  void onRemoveFromScene(Scene &scene) override;

private:
  ::physx::PxRigidStatic *mPxActor;
};

class PhysxRigidBodyComponent : public PhysxRigidBaseComponent {
public:
  Vec3 getLinearVelocity() const;
  Vec3 getAngularVelocity() const;

  float getMass() const;
  Vec3 getInertia() const;
  Pose getCMassLocalPose() const;
  void setMass(float m);
  void setInertia(Vec3 inertia);
  void setCMassLocalPose(Pose const &pose);

  bool getAutoComputeMass() const;
  void setAutoComputeMass(bool enable);

  virtual std::shared_ptr<PhysxRigidBaseComponent>
  attachCollision(std::shared_ptr<PhysxCollisionShape> shape) override;

  bool getDisableGravity() const;
  void setDisableGravity(bool disable);

  float getLinearDamping() const;
  void setLinearDamping(float damping);

  float getAngularDamping() const;
  void setAngularDamping(float damping);

  void addForceAtPoint(Vec3 const &force, Vec3 const &point, ::physx::PxForceMode::Enum mode);
  void addForceTorque(Vec3 const &force, Vec3 const &torque, ::physx::PxForceMode::Enum mode);

  // related to physx contact handling
  void setMaxDepenetrationVelocity(float speed);
  float getMaxDepenetrationVelocity() const;
  void setMaxContactImpulse(float impulse);
  float getMaxContactImpulse() const;

  ::physx::PxRigidBody *getPxActor() const override = 0;

protected:
  void internalUpdateMass();
  bool canAutoComputeMass();

  bool mAutoComputeMass{true};
  ::physx::PxMassProperties mMassProperties{0.f, ::physx::PxMat33(::physx::PxZero),
                                            ::physx::PxVec3(0.f)};
};

class PhysxRigidDynamicComponent : public PhysxRigidBodyComponent {
public:
  PhysxRigidDynamicComponent();
  void onSetPose(Pose const &pose) override;

  void setLinearVelocity(Vec3 const &v);
  void setAngularVelocity(Vec3 const &v);

  void setLockedMotionAxes(std::array<bool, 6> const &axes);
  std::array<bool, 6> getLockedMotionAxes() const;

  // void lockMotion(bool x, bool y, bool z, bool ax, bool ay, bool az);
  // std::array<bool, 6> lockedDofs() const;

  // TODO: expose solver iterations?
  // void setSolverIterations(uint32_t position, uint32_t velocity = 1);

  void setKinematicTarget(Pose const &pose);
  Pose getKinematicTarget() const;
  void setKinematic(bool kinematic);
  bool isKinematic() const;
  ::physx::PxRigidDynamic *getPxActor() const override { return mPxActor; }

  bool isSleeping() const;
  void wakeUp();
  void putToSleep();

  void onAddToScene(Scene &scene) override;
  void onRemoveFromScene(Scene &scene) override;

private:
  ::physx::PxRigidDynamic *mPxActor;
};

} // namespace physx
} // namespace sapien
