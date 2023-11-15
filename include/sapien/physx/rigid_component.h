#pragma once

#include "base_component.h"
#include "collision_shape.h"
#include "sapien/math/pose.h"
#include <PxPhysicsAPI.h>
#include <array>
#include <memory>
#include <string>

#include <cereal/access.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>
#include <cereal/types/vector.hpp>

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

  void afterStep();

  void internalRegisterJoint(std::shared_ptr<PhysxJointComponent> joint);
  void internalUnregisterJoint(std::shared_ptr<PhysxJointComponent> joint);

  /** called when some drive is deleted */
  void internalClearExpiredJoints();

  template <class Archive> void save(Archive &ar) const {
    ar(cereal::base_class<PhysxBaseComponent>(this));
    ar(mCollisionShapes);
  }

  template <class Archive> void load(Archive &ar) {
    ar(cereal::base_class<PhysxBaseComponent>(this));
    std::vector<std::shared_ptr<PhysxCollisionShape>> shapes;
    ar(shapes);
    for (auto s : shapes) {
      attachCollision(s);
    }
  }

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

  template <class Archive> void save(Archive &ar) const {
    ar(cereal::base_class<PhysxRigidBaseComponent>(this));
  }

  template <class Archive> void load(Archive &ar) {
    ar(cereal::base_class<PhysxRigidBaseComponent>(this));
  }

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

  template <class Archive> void save(Archive &ar) const {
    ar(cereal::base_class<PhysxRigidBaseComponent>(this));

    ::physx::PxActorFlags::InternalType actorFlags = getPxActor()->getActorFlags();
    ::physx::PxRigidBodyFlags::InternalType rigidBodyFlags = getPxActor()->getRigidBodyFlags();
    ar(actorFlags, rigidBodyFlags, getLinearDamping(), getAngularDamping(), getMaxDepenetrationVelocity(), getMaxContactImpulse());
    ar(getMass(), getInertia(), getCMassLocalPose());

    ar(mMassProperties.mass, mMassProperties.inertiaTensor.column0[0],
       mMassProperties.inertiaTensor.column0[1], mMassProperties.inertiaTensor.column0[2],
       mMassProperties.inertiaTensor.column1[0], mMassProperties.inertiaTensor.column1[1],
       mMassProperties.inertiaTensor.column1[2], mMassProperties.inertiaTensor.column2[0],
       mMassProperties.inertiaTensor.column2[1], mMassProperties.inertiaTensor.column2[2],
       mMassProperties.centerOfMass[0], mMassProperties.centerOfMass[1],
       mMassProperties.centerOfMass[2], mAutoComputeMass);
  }

  template <class Archive> void load(Archive &ar) {
    mAutoComputeMass = false;

    ar(cereal::base_class<PhysxRigidBaseComponent>(this));

    ::physx::PxActorFlags::InternalType actorFlags;
    ::physx::PxRigidBodyFlags::InternalType rigidBodyFlags;
    float linearDamping, angularDamping, speed, impulse;
    ar(actorFlags, rigidBodyFlags, linearDamping, angularDamping, speed, impulse);
    getPxActor()->setActorFlags(::physx::PxActorFlags(actorFlags));
    getPxActor()->setRigidBodyFlags(::physx::PxRigidBodyFlags(rigidBodyFlags));
    setLinearDamping(linearDamping);
    setAngularDamping(angularDamping);
    setMaxDepenetrationVelocity(speed);
    setMaxContactImpulse(impulse);

    float mass;
    Vec3 inertia;
    Pose localPose;
    ar(mass, inertia, localPose);
    setMass(mass);
    setInertia(inertia);
    setCMassLocalPose(localPose);

    ar(mMassProperties.mass, mMassProperties.inertiaTensor.column0[0],
       mMassProperties.inertiaTensor.column0[1], mMassProperties.inertiaTensor.column0[2],
       mMassProperties.inertiaTensor.column1[0], mMassProperties.inertiaTensor.column1[1],
       mMassProperties.inertiaTensor.column1[2], mMassProperties.inertiaTensor.column2[0],
       mMassProperties.inertiaTensor.column2[1], mMassProperties.inertiaTensor.column2[2],
       mMassProperties.centerOfMass[0], mMassProperties.centerOfMass[1],
       mMassProperties.centerOfMass[2], mAutoComputeMass);
  }

protected:
  void internalUpdateMass();
  bool canAutoComputeMass();

  bool mAutoComputeMass{true};
  ::physx::PxMassProperties mMassProperties{0.f, ::physx::PxMat33(::physx::PxZero), ::physx::PxVec3(0.f)};
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

  template <class Archive> void save(Archive &ar) const {
    ar(cereal::base_class<PhysxRigidBodyComponent>(this));
    ar(isKinematic());

    if (!isKinematic()) {
      ar(getLinearVelocity(), getAngularVelocity());
    }

    ::physx::PxRigidDynamicLockFlags::InternalType lockFlags = mPxActor->getRigidDynamicLockFlags();
    ar(lockFlags);
  }

  template <class Archive> void load(Archive &ar) {
    ar(cereal::base_class<PhysxRigidBodyComponent>(this));

    bool kinematic;
    ar(kinematic);
    setKinematic(kinematic);

    if (!kinematic) {
      Vec3 linearVelocity, angularVelocity;
      ar(linearVelocity, angularVelocity);
      setLinearVelocity(linearVelocity);
      setAngularVelocity(angularVelocity);
    }

    ::physx::PxRigidDynamicLockFlags::InternalType lockFlags;
    ar(lockFlags);
    mPxActor->setRigidDynamicLockFlags(static_cast<::physx::PxRigidDynamicLockFlags>(lockFlags));
  }

private:
  ::physx::PxRigidDynamic *mPxActor;
};

} // namespace physx
} // namespace sapien

CEREAL_REGISTER_TYPE(sapien::physx::PhysxRigidDynamicComponent);
CEREAL_REGISTER_TYPE(sapien::physx::PhysxRigidStaticComponent);

CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::physx::PhysxBaseComponent,
                                     sapien::physx::PhysxRigidBaseComponent);
CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::physx::PhysxRigidBaseComponent,
                                     sapien::physx::PhysxRigidBodyComponent);
CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::physx::PhysxRigidBaseComponent,
                                     sapien::physx::PhysxRigidStaticComponent);
CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::physx::PhysxRigidBodyComponent,
                                     sapien::physx::PhysxRigidDynamicComponent);
