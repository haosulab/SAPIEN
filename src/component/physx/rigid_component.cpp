#include "sapien/component/physx/rigid_component.h"
#include "sapien/component/physx/physx_system.h"
#include "sapien/entity.h"
#include "sapien/math/conversion.h"
#include "sapien/scene.h"

using namespace physx;

namespace sapien::component {

void PhysxRigidBaseComponent::afterStep() {
  // TODO: get on demand?
  getEntity()->internalSyncPose(PxTransformToPose(getPxActor()->getGlobalPose()));
}

std::shared_ptr<PhysxRigidBaseComponent>
PhysxRigidBaseComponent::attachCollision(std::shared_ptr<PhysxCollisionShape> shape) {
  getPxActor()->attachShape(*shape->getPxShape()); // TODO: set actor to shape
  mCollisionShapes.push_back(shape);
  shape->internalSetParent(this);
  return std::static_pointer_cast<PhysxRigidBaseComponent>(shared_from_this());
}

std::vector<std::shared_ptr<PhysxCollisionShape>>
PhysxRigidBaseComponent::getCollisionShapes() const {
  return mCollisionShapes;
}

AABB PhysxRigidBaseComponent::getGlobalAABBFast() const {
  auto shapes = getCollisionShapes();
  if (shapes.size() == 0) {
    throw std::runtime_error("afiled to get bounding box: no collision shapes attached");
  }
  AABB aabb = shapes.at(0)->getGlobalAABBFast();
  for (uint32_t i = 1; i < shapes.size(); ++i) {
    aabb = aabb + shapes[i]->getGlobalAABBFast();
  }
  return aabb;
}
AABB PhysxRigidBaseComponent::computeGlobalAABBTight() const {
  auto shapes = getCollisionShapes();
  if (shapes.size() == 0) {
    throw std::runtime_error("afiled to get bounding box: no collision shapes attached");
  }
  AABB aabb = shapes.at(0)->computeGlobalAABBTight();
  for (uint32_t i = 1; i < shapes.size(); ++i) {
    aabb = aabb + shapes[i]->computeGlobalAABBTight();
  }
  return aabb;
}

void PhysxRigidBaseComponent::internalRegisterJoint(std::shared_ptr<PhysxJointComponent> drive) {
  mJoints.push_back(drive);
}
void PhysxRigidBaseComponent::internalUnregisterJoint(std::shared_ptr<PhysxJointComponent> drive) {
  if (std::erase_if(mJoints, [drive](auto &d) { return d.lock() == drive; }) == 0) {
    throw std::runtime_error("failed to unregister drive: drive does not exist");
  }
}
void PhysxRigidBaseComponent::internalClearExpiredJoints() {
  std::erase_if(mJoints, [](auto &d) { return d.expired(); });
}

Vec3 PhysxRigidBodyComponent::getLinearVelocity() const {
  return PxVec3ToVec3(getPxActor()->getLinearVelocity());
}

Vec3 PhysxRigidBodyComponent::getAngularVelocity() const {
  return PxVec3ToVec3(getPxActor()->getAngularVelocity());
}

float PhysxRigidBodyComponent::getMass() const { return getPxActor()->getMass(); }
Vec3 PhysxRigidBodyComponent::getInertia() const {
  return PxVec3ToVec3(getPxActor()->getMassSpaceInertiaTensor());
}
Pose PhysxRigidBodyComponent::getCMassLocalPose() const {
  return PxTransformToPose(getPxActor()->getCMassLocalPose());
}

void PhysxRigidBodyComponent::setMass(float m) {
  getPxActor()->setMass(m);
  setAutoComputeMass(false);
}
void PhysxRigidBodyComponent::setInertia(Vec3 inertia) {
  getPxActor()->setMassSpaceInertiaTensor(Vec3ToPxVec3(inertia));
  setAutoComputeMass(false);
}
void PhysxRigidBodyComponent::setCMassLocalPose(Pose const &pose) {
  getPxActor()->setCMassLocalPose(PoseToPxTransform(pose));
  setAutoComputeMass(false);
}

void PhysxRigidDynamicComponent::setLinearVelocity(Vec3 const &v) {
  getPxActor()->setLinearVelocity(Vec3ToPxVec3(v));
}
void PhysxRigidDynamicComponent::setAngularVelocity(Vec3 const &v) {
  getPxActor()->setAngularVelocity(Vec3ToPxVec3(v));
}

void PhysxRigidDynamicComponent::setLockedMotionAxes(std::array<bool, 6> const &axes) {
  getPxActor()->setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_LINEAR_X, axes[0]);
  getPxActor()->setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_LINEAR_Y, axes[1]);
  getPxActor()->setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_LINEAR_Z, axes[2]);
  getPxActor()->setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_ANGULAR_X, axes[3]);
  getPxActor()->setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y, axes[4]);
  getPxActor()->setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z, axes[5]);
}
std::array<bool, 6> PhysxRigidDynamicComponent::getLockedMotionAxes() const {
  auto flags = getPxActor()->getRigidDynamicLockFlags();
  return {flags.isSet(PxRigidDynamicLockFlag::eLOCK_LINEAR_X),
          flags.isSet(PxRigidDynamicLockFlag::eLOCK_LINEAR_Y),
          flags.isSet(PxRigidDynamicLockFlag::eLOCK_LINEAR_Z),
          flags.isSet(PxRigidDynamicLockFlag::eLOCK_ANGULAR_X),
          flags.isSet(PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y),
          flags.isSet(PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z)};
}

void PhysxRigidDynamicComponent::setKinematicTarget(Pose const &pose) {
  if (!isKinematic()) {
    throw std::runtime_error("failed to set kinematic target: actor is not kinematic");
  }
  getPxActor()->setKinematicTarget(PoseToPxTransform(pose));
}

Pose PhysxRigidDynamicComponent::getKinematicTarget() const {
  if (!isKinematic()) {
    throw std::runtime_error("failed to set kinematic target: actor is not kinematic");
  }
  PxTransform target;
  if (getPxActor()->getKinematicTarget(target)) {
    return PxTransformToPose(target);
  }
  throw std::runtime_error("failed to set kinematic target: target not set");
}

void PhysxRigidDynamicComponent::setKinematic(bool kinematic) {
  getPxActor()->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, kinematic);
  if (!kinematic && getAutoComputeMass()) {
    // TODO: warn that mass will not be auto computed
    setAutoComputeMass(false);
  }
}
bool PhysxRigidDynamicComponent::isKinematic() const {
  return getPxActor()->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC);
}

bool PhysxRigidDynamicComponent::isSleeping() const { return getPxActor()->isSleeping(); }
void PhysxRigidDynamicComponent::wakeUp() { getPxActor()->wakeUp(); }
void PhysxRigidDynamicComponent::putToSleep() { getPxActor()->putToSleep(); }

//========== lifecycle methods ==========//

PhysxRigidStaticComponent::PhysxRigidStaticComponent() {
  mPxActor = PhysxEngine::Get()->getPxPhysics()->createRigidStatic(PxTransform{PxIdentity});
  mPxActor->userData = this;
}

PhysxRigidDynamicComponent::PhysxRigidDynamicComponent() {
  mPxActor = PhysxEngine::Get()->getPxPhysics()->createRigidDynamic(PxTransform{PxIdentity});
  mPxActor->userData = this;
  mPxActor->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_GYROSCOPIC_FORCES, true);
  internalUpdateMass();
}

void PhysxRigidStaticComponent::onSetPose(Pose const &pose) {
  getPxActor()->setGlobalPose(PoseToPxTransform(pose));
}

void PhysxRigidDynamicComponent::onSetPose(Pose const &pose) {
  getPxActor()->setGlobalPose(PoseToPxTransform(pose));
}

void PhysxRigidStaticComponent::onAddToScene(Scene &scene) {
  auto system = scene.getPhysxSystem();
  system->getPxScene()->addActor(*getPxActor());
  system->registerComponent(
      std::static_pointer_cast<PhysxRigidStaticComponent>(shared_from_this()));

  // setup physical parameters
  for (auto &shape : mCollisionShapes) {
    shape->getPxShape()->setContactOffset(system->getSceneConfig().contactOffset);
  }
}

void PhysxRigidStaticComponent::onRemoveFromScene(Scene &scene) {
  auto system = scene.getPhysxSystem();
  system->getPxScene()->removeActor(*getPxActor());

  system->unregisterComponent(
      std::static_pointer_cast<PhysxRigidStaticComponent>(shared_from_this()));
}

void PhysxRigidDynamicComponent::onAddToScene(Scene &scene) {
  auto system = scene.getPhysxSystem();
  system->getPxScene()->addActor(*getPxActor());

  // setup physical parameters
  mPxActor->setSolverIterationCounts(system->getSceneConfig().solverIterations,
                                     system->getSceneConfig().solverVelocityIterations);
  mPxActor->setSleepThreshold(system->getSceneConfig().sleepThreshold);
  for (auto &shape : mCollisionShapes) {
    shape->getPxShape()->setContactOffset(system->getSceneConfig().contactOffset);
  }

  system->registerComponent(
      std::static_pointer_cast<PhysxRigidDynamicComponent>(shared_from_this()));
}

void PhysxRigidDynamicComponent::onRemoveFromScene(Scene &scene) {
  auto system = scene.getPhysxSystem();
  system->getPxScene()->removeActor(*getPxActor());

  system->unregisterComponent(
      std::static_pointer_cast<PhysxRigidDynamicComponent>(shared_from_this()));
}

std::shared_ptr<PhysxRigidBaseComponent>
PhysxRigidBodyComponent::attachCollision(std::shared_ptr<PhysxCollisionShape> shape) {
  getPxActor()->attachShape(*shape->getPxShape());
  mCollisionShapes.push_back(shape);
  shape->internalSetParent(this);

  if (std::dynamic_pointer_cast<PhysxCollisionShapePlane>(shape) ||
      std::dynamic_pointer_cast<PhysxCollisionShapeTriangleMesh>(shape)) {
    setAutoComputeMass(false);
  }
  if (getAutoComputeMass()) {
    auto s = shape->getPxShape();
    auto m = PxRigidBodyExt::computeMassPropertiesFromShapes(&s, 1) * shape->getDensity();
    std::vector<PxMassProperties> mp = {mMassProperties, m};
    std::vector<PxTransform> t = {PxTransform{PxIdentity}, PxTransform{PxIdentity}};
    mMassProperties = PxMassProperties::sum(mp.data(), t.data(), 2);
    internalUpdateMass();
  }
  return std::static_pointer_cast<PhysxRigidBaseComponent>(shared_from_this());
}

bool PhysxRigidBodyComponent::getDisableGravity() const {
  return getPxActor()->getActorFlags() & PxActorFlag::eDISABLE_GRAVITY;
}

void PhysxRigidBodyComponent::setDisableGravity(bool disable) {
  getPxActor()->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, disable);
}

void PhysxRigidBodyComponent::internalUpdateMass() {
  PxQuat q;
  PxVec3 I = PxDiagonalize(mMassProperties.inertiaTensor, q);

  getPxActor()->setMass(std::max(mMassProperties.mass, 1e-6f));
  getPxActor()->setMassSpaceInertiaTensor(
      {std::max(I.x, 1e-6f), std::max(I.y, 1e-6f), std::max(I.z, 1e-6f)});
  getPxActor()->setCMassLocalPose({mMassProperties.centerOfMass, q});
}

bool PhysxRigidBodyComponent::canAutoComputeMass() {
  for (auto c : mCollisionShapes) {
    auto type = c->getPxShape()->getGeometry().getType();
    if (type == PxGeometryType::eTRIANGLEMESH || type == PxGeometryType::eHEIGHTFIELD ||
        type == PxGeometryType::ePLANE) {
      return false;
    }
  }
  return true;
}

bool PhysxRigidBodyComponent::getAutoComputeMass() const { return mAutoComputeMass; }
void PhysxRigidBodyComponent::setAutoComputeMass(bool enable) {
  if (enable) {
    if (!canAutoComputeMass()) {
      throw std::runtime_error("failed to auto compute mass: attached shapes must be box, sphere, "
                               "capsule, or convex mesh");
    }
    // TODO: finish this
    throw std::runtime_error("compute mass is not implemented yet");
  }
  mAutoComputeMass = enable;
}

float PhysxRigidBodyComponent::getLinearDamping() const {
  return getPxActor()->getLinearDamping();
}
void PhysxRigidBodyComponent::setLinearDamping(float damping) {
  getPxActor()->setLinearDamping(damping);
}

float PhysxRigidBodyComponent::getAngularDamping() const {
  return getPxActor()->getAngularDamping();
}
void PhysxRigidBodyComponent::setAngularDamping(float damping) {
  getPxActor()->setAngularDamping(damping);
}

void PhysxRigidBodyComponent::addForceAtPoint(Vec3 const &force, Vec3 const &point,
                                              PxForceMode::Enum mode) {
  PxRigidBodyExt::addForceAtPos(*getPxActor(), Vec3ToPxVec3(force), Vec3ToPxVec3(point), mode);
}

void PhysxRigidBodyComponent::addForceTorque(Vec3 const &force, Vec3 const &torque,
                                             PxForceMode::Enum mode) {
  getPxActor()->addForce(Vec3ToPxVec3(force), mode);
  getPxActor()->addTorque(Vec3ToPxVec3(torque), mode);
}

void PhysxRigidBodyComponent::setMaxDepenetrationVelocity(float speed) {
  getPxActor()->setMaxDepenetrationVelocity(speed);
}
float PhysxRigidBodyComponent::getMaxDepenetrationVelocity() const {
  return getPxActor()->getMaxDepenetrationVelocity();
}
void PhysxRigidBodyComponent::setMaxContactImpulse(float impulse) {
  getPxActor()->setMaxContactImpulse(impulse);
}
float PhysxRigidBodyComponent::getMaxContactImpulse() const {
  return getPxActor()->getMaxContactImpulse();
}

} // namespace sapien::component
