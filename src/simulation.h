#pragma once
#include "filter_shader.h"
#include "id_generator.h"
#include "mesh_manager.h"
#include "render_interface.h"
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
class SScene;

struct SimulationSave {
  std::string name;
  std::vector<PxReal> data;
};

class SapienErrorCallback : public PxErrorCallback {
  PxErrorCode::Enum mLastErrorCode = PxErrorCode::eNO_ERROR;

public:
  virtual void reportError(PxErrorCode::Enum code, const char *message, const char *file,
                           int line) override;
  PxErrorCode::Enum getLastErrorCode();
};

class Simulation {
public:
  PxPhysics *mPhysicsSDK = nullptr;
  PxFoundation *mFoundation = nullptr;
  PxCooking *mCooking = nullptr;

  SapienErrorCallback mErrorCallback;

  // std::vector<PxRigidActor *> mRigidActors;
  // PxScene *mScene = nullptr;
  // PxReal mTimestep = 1.0f / 60.0f;

private:
  uint32_t mThreadCount = 0;
  Renderer::IPxrRenderer *mRenderer = nullptr;
  PxDefaultCpuDispatcher *mCpuDispatcher = nullptr;

public:
  PxMaterial *mDefaultMaterial = nullptr;
  // CollisionGroupManager collisionManager;

  //=== mesh managers ===//
private:
  MeshManager mMeshManager;

public:
  inline MeshManager &getMeshManager() { return mMeshManager; }

  //=== scenes ===//
public:
  std::unique_ptr<SScene> createScene(PxVec3 gravity = {0.f, 0.f, -9.81f},
                                      PxSolverType::Enum solverType = PxSolverType::ePGS,
                                      PxSceneFlags sceneFlags = PxSceneFlags());

#ifdef _PVD
  PxPvd *mPvd = nullptr;
  PxPvdTransport *mTransport = nullptr;
#endif

  // std::map<physx_id_t, PxTransform> mCameraId2InitialPose;
  // std::map<physx_id_t, PxRigidActor *> mMountedCamera2MountedActor;

  // std::map<physx_id_t, PxTransform> mRenderId2InitialPose;
  // std::map<physx_id_t, PxRigidActor *> mRenderId2Actor;
  // std::map<physx_id_t, std::string> mRenderId2VisualName;

  // std::map<physx_id_t, PxRigidActor *> mLinkId2Actor;
  // std::map<PxRigidActor *, physx_id_t> mActor2LinkId;
  // std::map<physx_id_t, IArticulationBase *> mLinkId2Articulation;

  // std::vector<std::unique_ptr<struct ArticulationWrapper>> mDynamicArticulationWrappers;
  // std::vector<std::unique_ptr<class KinematicsArticulationWrapper>>
  // mKinematicArticulationWrappers; std::vector<std::unique_ptr<class JointSystem>>
  // mJointSystemWrappers; std::vector<std::unique_ptr<class ControllableArticulationWrapper>>
  //     mControllableArticulationWrapper;

  // private:
  //   std::vector<std::function<void(Simulation &)>> mStepCallBacks;

public:
  Simulation(uint32_t nthread = 0);
  ~Simulation();

  void setRenderer(Renderer::IPxrRenderer *renderer);
  inline Renderer::IPxrRenderer *getRenderer() { return mRenderer; }

  // std::unique_ptr<class ActorBuilder> createActorBuilder();
  // std::unique_ptr<class ArticulationBuilder> createArticulationBuilder();
  PxMaterial *createPhysicalMaterial(PxReal staticFriction, PxReal dynamicFriction,
                                     PxReal restitution) const;

  // physx_id_t addMountedCamera(std::string const &name, PxRigidActor *actor,
  //                             PxTransform const &pose, uint32_t width, uint32_t height, float
  //                             fovx, float fovy, float near = 0.1, float far = 100);

  /* Functions related to simulation steps */

  // void step();
  // std::vector<PxReal> dump();
  // void pack(const std::vector<PxReal> &data);
  // inline void bindStepCallBack(const std::function<void(Simulation const &)> &callBack) {
  //   mStepCallBacks.emplace_back(callBack);
  // };

  /* Sync with renderer by calling UpdateRigidbody */
  // void updateRenderer();
  // PxRigidStatic *addGround(PxReal altitude, bool render = true, PxMaterial *material = nullptr);

  /* Create URDF loader from simulation */
  // std::unique_ptr<URDF::URDFLoader> createURDFLoader();

  /* Create Controllable wrapper and register update function */
  // class ControllableArticulationWrapper *
  // createControllableArticulationWrapper(class IArticulationDrivable *baseWrapper);
  // void clearCache();

  // For internal use only for now
  // std::vector<SimulationSave> simulationSaves;
  // void writeSavesToDisk(const std::string &filename);
  // void loadSavesFromDisk(const std::string &filename);
  // inline void appendSaves(std::string const &name) { simulationSaves.push_back({name, dump()});
  // } inline void deleteSave(uint32_t index) {
  //   if (index < simulationSaves.size()) {
  //     simulationSaves.erase(simulationSaves.begin() + index);
  //   }
  // }
  // inline void loadSave(uint32_t index) {
  //   if (index < simulationSaves.size()) {
  //     pack(simulationSaves[index].data);
  //     step();
  //     pack(simulationSaves[index].data);
  //   }
  // }
};

} // namespace sapien
