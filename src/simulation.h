#pragma once
#include "articulation_interface.h"
#include "filter_shader.h"
#include "id_generator.h"
#include "joint_system.h"
#include "render_interface.h"
#include "urdf/urdf_loader.h"
#include <PxPhysicsAPI.h>
#include <extensions/PxDefaultAllocator.h>
#include <extensions/PxDefaultCpuDispatcher.h>
#include <extensions/PxDefaultErrorCallback.h>
#include <extensions/PxDefaultSimulationFilterShader.h>
#include <extensions/PxExtensionsAPI.h>
#include <extensions/PxShapeExt.h>
#include <extensions/PxSimpleFactory.h>
#include <foundation/PxMat33.h>
#include <iostream>
#include <map>
#include <memory>
#include <vector>

namespace sapien {
using namespace physx;

class Simulation {
public:
  std::vector<PxRigidActor *> mRigidActors;
  PxPhysics *mPhysicsSDK = nullptr;
  PxFoundation *mFoundation = nullptr;
  PxCooking *mCooking = nullptr;
  PxScene *mScene = nullptr;
  PxReal mTimestep = 1.0f / 60.0f;
  Renderer::IPhysxRenderer *mRenderer = nullptr;
  PxDefaultCpuDispatcher *mCpuDispatcher = nullptr;
  PxMaterial *mDefaultMaterial = nullptr;
  CollisionGroupManager collisionManager;

#ifdef _PVD
  PxPvd *mPvd = nullptr;
  PxPvdTransport *mTransport = nullptr;
#endif
  std::map<physx_id_t, PxTransform> mCameraId2InitialPose;
  std::map<physx_id_t, PxRigidActor *> mMountedCamera2MountedActor;

  std::map<physx_id_t, PxTransform> mRenderId2InitialPose;
  std::map<physx_id_t, PxRigidActor *> mRenderId2Actor;
  std::map<physx_id_t, std::string> mRenderId2VisualName;

  std::map<physx_id_t, PxRigidActor *> mLinkId2Actor;
  std::map<PxRigidActor *, physx_id_t> mActor2LinkId;
  std::map<physx_id_t, IArticulationBase *> mLinkId2Articulation;

  std::vector<std::unique_ptr<struct ArticulationWrapper>> mDynamicArticulationWrappers;
  std::vector<std::unique_ptr<class KinematicsArticulationWrapper>> mKinematicArticulationWrappers;
  std::vector<std::unique_ptr<class JointSystem>> mJointSystemWrappers;
  std::vector<std::unique_ptr<class ControllableArticulationWrapper>>
      mControllableArticulationWrapper;

private:
  std::vector<std::function<void(Simulation &)>> mStepCallBacks;

public:
  Simulation();
  ~Simulation();

  inline void setTimestep(PxReal step) { mTimestep = step; };
  inline PxReal getTimestep() const { return mTimestep; };

  void setRenderer(Renderer::IPhysxRenderer *renderer);
  inline Renderer::IPhysxRenderer *getRenderer() { return mRenderer; }

  std::unique_ptr<class ActorBuilder> createActorBuilder();
  std::unique_ptr<class ArticulationBuilder> createArticulationBuilder();
  PxMaterial *createPhysicalMaterial(PxReal staticFriction, PxReal dynamicFriction,
                                     PxReal restitution) const;

  physx_id_t addMountedCamera(std::string const &name, PxRigidActor *actor,
                              PxTransform const &pose, uint32_t width, uint32_t height, float fovx,
                              float fovy, float near = 0.1, float far = 100);

  /* Functions related to simulation steps */
  void step();
  std::vector<PxReal> dump();
  void pack(const std::vector<PxReal> &data);
  inline void bindStepCallBack(const std::function<void(Simulation const &)> &callBack) {
    mStepCallBacks.emplace_back(callBack);
  };

  /* Sync with renderer by calling UpdateRigidbody */
  void updateRenderer();
  PxRigidStatic *addGround(PxReal altitude, bool render = true, PxMaterial *material = nullptr);

  /* Create URDF loader from simulation */
  std::unique_ptr<URDF::URDFLoader> createURDFLoader();

  /* Create Controllable wrapper and register update function */
  class ControllableArticulationWrapper *
  createControllableArticulationWrapper(class IArticulationDrivable *baseWrapper);
  void clearCache();
};

struct SimulationCache {
  std::vector<std::vector<PxReal>> cache;
  std::vector<std::string> frameNames;
  Simulation sim;
  SimulationCache(Simulation &sim) : sim(sim){};
  inline void pack(size_t i) {
    assert(i < cache.size());
    sim.pack(cache[i]);
  }
  inline void pack(const std::string &name) {
    size_t i = std::find(frameNames.begin(), frameNames.end(), name) - frameNames.begin();
    assert(i < cache.size());
    sim.pack(cache[i]);
  }
  inline void dump(const std::string &name = "") {
    std::string frameName = name.empty() ? std::to_string(cache.size()) : name;
    cache.push_back(sim.dump());
  }
  bool save(const std::string &filename);
  bool load(const std::string &filename);
};

struct CacheHeader {
  uint32_t numArticulation = 0;
  uint32_t totalDOF = 0;
  uint32_t dimension;
  std::vector<std::vector<std::string>> jointNames;
  explicit CacheHeader(Simulation &sim);
  void save()
};

} // namespace sapien
