#include "sapien/component/physx/physx_system.h"
#include "../../logger.h"
#include "./filter_shader.hpp"
#include "sapien/component/physx/articulation.h"
#include "sapien/component/physx/articulation_link_component.h"
#include "sapien/component/physx/material.h"
#include "sapien/component/physx/rigid_component.h"
#include "sapien/math/conversion.h"
#include <extensions/PxExtensionsAPI.h>

using namespace physx;
namespace sapien {
namespace component {

static PxDefaultAllocator gDefaultAllocatorCallback;

class SapienErrorCallback : public PxErrorCallback {
  physx::PxErrorCode::Enum mLastErrorCode = PxErrorCode::eNO_ERROR;

public:
  void reportError(PxErrorCode::Enum code, const char *message, const char *file,
                   int line) override {
    mLastErrorCode = code;

#ifdef NDEBUG
    logger::critical("{}", message);
#else
    logger::critical("{}:{}: {}", file, line, message);
#endif
  }
  PxErrorCode::Enum getLastErrorCode() {
    auto code = mLastErrorCode;
    mLastErrorCode = PxErrorCode::eNO_ERROR;
    return code;
  }
};

static SapienErrorCallback gDefaultErrorCallback;

static std::weak_ptr<PhysxEngine> gEngine;
std::shared_ptr<PhysxEngine> PhysxEngine::Get(float toleranceLength, float toleranceSpeed) {
  auto engine = gEngine.lock();
  if (!engine) {
    gEngine = engine = std::make_shared<PhysxEngine>(toleranceLength, toleranceSpeed);
  }
  return engine;
}

PhysxEngine::PhysxEngine(float toleranceLength, float toleranceSpeed) {
  logger::getLogger(); // init logger

  mPxFoundation =
      PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
  if (!mPxFoundation) {
    throw std::runtime_error("PhysX foundation creation failed");
  }

  PxTolerancesScale toleranceScale(toleranceLength, toleranceSpeed);

  mPxPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mPxFoundation, toleranceScale);
  if (!mPxPhysics) {
    throw std::runtime_error("PhysX creation failed");
  }
  if (!PxInitExtensions(*mPxPhysics, nullptr)) {
    throw std::runtime_error("PhysX extension initialization failed");
  }

  // mMeshManager = std::make_unique<MeshManager>();
}

PhysxEngine::~PhysxEngine() {
  // mMeshManager.reset();
  PxCloseExtensions();
  mPxPhysics->release();
  mPxFoundation->release();
}

PhysxSystem::PhysxSystem(PhysxSceneConfig const &config)
    : mSceneConfig(config), mEngine(PhysxEngine::Get()) {
  PxSceneDesc sceneDesc(mEngine->getPxPhysics()->getTolerancesScale());
  sceneDesc.gravity = Vec3ToPxVec3(config.gravity);
  sceneDesc.filterShader = TypeAffinityIgnoreFilterShader;
  sceneDesc.solverType = config.enableTGS ? PxSolverType::eTGS : PxSolverType::ePGS;
  sceneDesc.bounceThresholdVelocity = config.bounceThreshold;

  PxSceneFlags sceneFlags;
  if (config.enableEnhancedDeterminism) {
    sceneFlags |= PxSceneFlag::eENABLE_ENHANCED_DETERMINISM;
  }
  if (config.enablePCM) {
    sceneFlags |= PxSceneFlag::eENABLE_PCM;
  }
  if (config.enableCCD) {
    sceneFlags |= PxSceneFlag::eENABLE_CCD;
  }
  if (config.enableFrictionEveryIteration) {
    sceneFlags |= PxSceneFlag::eENABLE_FRICTION_EVERY_ITERATION;
  }
  sceneDesc.flags = sceneFlags;

  mPxCPUDispatcher = PxDefaultCpuDispatcherCreate(0);
  if (!mPxCPUDispatcher) {
    throw std::runtime_error("PhysX system creation failed: failed to create CPU dispatcher");
  }
  sceneDesc.cpuDispatcher = mPxCPUDispatcher;

  mPxScene = mEngine->getPxPhysics()->createScene(sceneDesc);

  mPxScene->setSimulationEventCallback(&mSimulationCallback);
}

void PhysxSystem::registerComponent(
    std::shared_ptr<component::PhysxRigidDynamicComponent> component) {
  mRigidDynamicComponents.insert(component);
}
void PhysxSystem::registerComponent(
    std::shared_ptr<component::PhysxRigidStaticComponent> component) {
  mRigidStaticComponents.insert(component);
}
void PhysxSystem::registerComponent(
    std::shared_ptr<component::PhysxArticulationLinkComponent> component) {
  mArticulationLinkComponents.insert(component);
}

void PhysxSystem::unregisterComponent(
    std::shared_ptr<component::PhysxRigidDynamicComponent> component) {
  mRigidDynamicComponents.erase(component);
}
void PhysxSystem::unregisterComponent(
    std::shared_ptr<component::PhysxRigidStaticComponent> component) {
  mRigidStaticComponents.erase(component);
}
void PhysxSystem::unregisterComponent(
    std::shared_ptr<component::PhysxArticulationLinkComponent> component) {
  mArticulationLinkComponents.erase(component);
}

std::vector<std::shared_ptr<component::PhysxRigidDynamicComponent>>
PhysxSystem::getRigidDynamicComponents() const {
  return {mRigidDynamicComponents.begin(), mRigidDynamicComponents.end()};
}
std::vector<std::shared_ptr<component::PhysxRigidStaticComponent>>
PhysxSystem::getRigidStaticComponents() const {
  return {mRigidStaticComponents.begin(), mRigidStaticComponents.end()};
}
std::vector<std::shared_ptr<component::PhysxArticulationLinkComponent>>
PhysxSystem::getArticulationLinkComponents() const {
  return {mArticulationLinkComponents.begin(), mArticulationLinkComponents.end()};
}

std::unique_ptr<PhysxHitInfo> PhysxSystem::raycast(Vec3 const &origin, Vec3 const &direction,
                                                   float distance) {
  PxRaycastBuffer hit;
  bool status = mPxScene->raycast(Vec3ToPxVec3(origin), Vec3ToPxVec3(direction), distance, hit);
  if (status) {
    return std::make_unique<PhysxHitInfo>(
        PxVec3ToVec3(hit.block.position), PxVec3ToVec3(hit.block.normal), hit.block.distance,
        static_cast<PhysxCollisionShape *>(hit.block.shape->userData),
        static_cast<PhysxRigidBaseComponent *>(hit.block.actor->userData));
  }
  return nullptr;
}

void PhysxSystem::step() {
  mPxScene->simulate(mTimestep);
  while (!mPxScene->fetchResults(true)) {
  }

  // sync phsyx pose to SAPIEN objects
  for (auto c : mRigidStaticComponents) {
    c->afterStep();
  }
  for (auto c : mRigidDynamicComponents) {
    c->afterStep();
  }
  for (auto c : mArticulationLinkComponents) {
    c->afterStep();
  }
}

std::string PhysxSystem::packState() const {
  std::ostringstream ss;
  for (auto &actor : mRigidDynamicComponents) {
    Pose pose = actor->getPose();
    Vec3 v = actor->getLinearVelocity();
    Vec3 w = actor->getAngularVelocity();
    ss.write(reinterpret_cast<const char *>(&pose), sizeof(Pose));
    ss.write(reinterpret_cast<const char *>(&v), sizeof(Vec3));
    ss.write(reinterpret_cast<const char *>(&w), sizeof(Vec3));
  }
  for (auto &link : mArticulationLinkComponents) {
    if (link->isRoot()) {
      auto art = link->getArticulation();

      Pose pose = art->getRootPose();
      Vec3 v = art->getRootLinearVelocity();
      Vec3 w = art->getRootAngularVelocity();
      ss.write(reinterpret_cast<const char *>(&pose), sizeof(Pose));
      ss.write(reinterpret_cast<const char *>(&v), sizeof(Vec3));
      ss.write(reinterpret_cast<const char *>(&w), sizeof(Vec3));

      auto qpos = art->getQpos();
      auto qvel = art->getQvel();

      ss.write(reinterpret_cast<const char *>(qpos.data()), qpos.size() * sizeof(float));
      ss.write(reinterpret_cast<const char *>(qvel.data()), qvel.size() * sizeof(float));

      for (auto j : art->getActiveJoints()) {
        auto pos = j->getDriveTargetPosition();
        auto vel = j->getDriveTargetVelocity();

        ss.write(reinterpret_cast<const char *>(pos.data()), pos.size() * sizeof(float));
        ss.write(reinterpret_cast<const char *>(vel.data()), vel.size() * sizeof(float));

        // float p = j->getDriveStiffness();
        // float d = j->getDriveDamping();
        // float l = j->getDriveForceLimit();
        // int m = j->getDriveType();

        // ss.write(reinterpret_cast<const char *>(&p), sizeof(float));
        // ss.write(reinterpret_cast<const char *>(&d), sizeof(float));
        // ss.write(reinterpret_cast<const char *>(&l), sizeof(float));
        // ss.write(reinterpret_cast<const char *>(&m), sizeof(int));
      }
    }
  }
  return ss.str();
}

void PhysxSystem::unpackState(std::string const &data) {
  std::istringstream ss(data);
  for (auto &actor : mRigidDynamicComponents) {
    Pose pose;
    Vec3 v, w;
    ss.read(reinterpret_cast<char *>(&pose), sizeof(Pose));
    ss.read(reinterpret_cast<char *>(&v), sizeof(Vec3));
    ss.read(reinterpret_cast<char *>(&w), sizeof(Vec3));
    actor->setPose(pose);
    actor->setLinearVelocity(v);
    actor->setAngularVelocity(w);
  }
  for (auto &link : mArticulationLinkComponents) {
    if (link->isRoot()) {
      Pose pose;
      Vec3 v, w;
      ss.read(reinterpret_cast<char *>(&pose), sizeof(Pose));
      ss.read(reinterpret_cast<char *>(&v), sizeof(Vec3));
      ss.read(reinterpret_cast<char *>(&w), sizeof(Vec3));
      auto art = link->getArticulation();
      art->setRootPose(pose);
      art->setRootLinearVelocity(v);
      art->setRootAngularVelocity(w);

      Eigen::VectorXf qpos;
      Eigen::VectorXf qvel;
      qpos.resize(art->getDof());
      qvel.resize(art->getDof());
      ss.read(reinterpret_cast<char *>(qpos.data()), qpos.size() * sizeof(float));
      ss.read(reinterpret_cast<char *>(qvel.data()), qvel.size() * sizeof(float));
      art->setQpos(qpos);
      art->setQvel(qvel);

      for (auto j : art->getActiveJoints()) {
        Eigen::VectorXf pos, vel;
        pos.resize(j->getDof());
        vel.resize(j->getDof());
        ss.read(reinterpret_cast<char *>(pos.data()), pos.size() * sizeof(float));
        ss.read(reinterpret_cast<char *>(vel.data()), vel.size() * sizeof(float));
        j->setDriveTargetPosition(pos);
        j->setDriveTargetVelocity(vel);

        // float p, d, l;
        // int m;
        // ss.read(reinterpret_cast<char *>(&p), sizeof(float));
        // ss.read(reinterpret_cast<char *>(&d), sizeof(float));
        // ss.read(reinterpret_cast<char *>(&l), sizeof(float));
        // ss.read(reinterpret_cast<char *>(&m), sizeof(int));

        // j->setDriveProperties(p, d, l, static_cast<physx::PxArticulationDriveType::Enum>(m));
      }
    }
  }
  // TODO: check nothing is left
}

} // namespace component
} // namespace sapien
