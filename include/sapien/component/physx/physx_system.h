#pragma once
#include "../system.h"
#include "mesh_manager.h"
#include "sapien/scene.h"
#include "simulation_callback.hpp"
#include <PxPhysicsAPI.h>
#include <memory>
#include <unordered_set>

namespace sapien {
namespace component {
class PhysxArticulation;
class PhysxMaterial;

struct PhysxSceneConfig {
  Vec3 gravity = {0, 0, -9.81};           // default gravity
  float static_friction = 0.3f;           // default static friction coefficient
  float dynamic_friction = 0.3f;          // default dynamic friction coefficient
  float restitution = 0.1f;               // default restitution coefficient
  float bounceThreshold = 2.f;            // relative velocity below this will not bounce
  float sleepThreshold = 0.005f;          // put to sleep if (kinetic energy/(mass) falls below
  float contactOffset = 0.01f;            // how close should contacts be generated
  uint32_t solverIterations = 10;         // solver position iterations, helps reduce jittering
  uint32_t solverVelocityIterations = 1;  // solver velocity iterations
  bool enablePCM = false;                 // Use persistent contact manifold solver for contact
  bool enableTGS = false;                 // use TGS solver
  bool enableCCD = false;                 // use continuous collision detection
  bool enableEnhancedDeterminism = false; // improve determinism
  bool enableFrictionEveryIteration =
      true; // better friction calculation, recommended for robotics

  template <class Archive> void serialize(Archive &ar) {
    ar(gravity, static_friction, dynamic_friction, restitution, bounceThreshold, sleepThreshold,
       contactOffset, solverIterations, solverVelocityIterations, enablePCM, enableTGS, enableCCD,
       enableEnhancedDeterminism, enableFrictionEveryIteration);
  }
};

class PhysxEngine {
public:
  static std::shared_ptr<PhysxEngine> Get(float toleranceLength = 0.1f,
                                          float toleranceSpeed = 0.2f);
  PhysxEngine(float toleranceLength, float toleranceSpeed);
  physx::PxPhysics *getPxPhysics() const { return mPxPhysics; }

  ~PhysxEngine();

private:
  physx::PxPhysics *mPxPhysics;
  physx::PxFoundation *mPxFoundation;
};

class PhysxSystem : public System {
public:
  PhysxSystem(PhysxSceneConfig const &config = {});
  std::shared_ptr<PhysxEngine> getEngine() const { return mEngine; }
  physx::PxScene *getPxScene() const { return mPxScene; }

  // register component so that it is updated by step
  void registerComponent(std::shared_ptr<component::PhysxRigidDynamicComponent> component);
  void registerComponent(std::shared_ptr<component::PhysxRigidStaticComponent> component);
  void registerComponent(std::shared_ptr<component::PhysxArticulationLinkComponent> component);
  void unregisterComponent(std::shared_ptr<component::PhysxRigidDynamicComponent> component);
  void unregisterComponent(std::shared_ptr<component::PhysxRigidStaticComponent> component);
  void unregisterComponent(std::shared_ptr<component::PhysxArticulationLinkComponent> component);

  std::vector<std::shared_ptr<component::PhysxRigidDynamicComponent>>
  getRigidDynamicComponents() const;
  std::vector<std::shared_ptr<component::PhysxRigidStaticComponent>>
  getRigidStaticComponents() const;
  std::vector<std::shared_ptr<component::PhysxArticulationLinkComponent>>
  getArticulationLinkComponents() const;

  std::string packState() const;
  void unpackState(std::string const &data);

  void setTimestep(float step) { mTimestep = step; }
  float getTimestep() { return mTimestep; }

  void step() override;
  std::string getName() const override { return "physx"; }

  std::vector<Contact *> getContacts() const { return mSimulationCallback.getContacts(); }

  PhysxSceneConfig const &getSceneConfig() const { return mSceneConfig; };

  template <class Archive> void save(Archive &ar) const { ar(mSceneConfig); }
  template <class Archive>
  static void load_and_construct(Archive &ar, cereal::construct<PhysxSystem> &construct) {
    PhysxSceneConfig config;
    ar(config);
    construct(config);
  }

private:
  PhysxSceneConfig mSceneConfig;

  std::shared_ptr<PhysxEngine> mEngine;
  physx::PxDefaultCpuDispatcher *mPxCPUDispatcher;
  physx::PxScene *mPxScene;
  float mTimestep{0.01f};

  std::unordered_set<std::shared_ptr<component::PhysxRigidDynamicComponent>>
      mRigidDynamicComponents;
  std::unordered_set<std::shared_ptr<component::PhysxRigidStaticComponent>> mRigidStaticComponents;
  std::unordered_set<std::shared_ptr<component::PhysxArticulationLinkComponent>>
      mArticulationLinkComponents;

  DefaultEventCallback mSimulationCallback;
};

} // namespace component
} // namespace sapien

CEREAL_REGISTER_TYPE(sapien::component::PhysxSystem);
CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::component::System, sapien::component::PhysxSystem);
