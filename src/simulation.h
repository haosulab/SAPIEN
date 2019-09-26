#pragma once
#include "filter_shader.h"
#include "id_generator.h"
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
#include "id_generator.h"
#include "filter_shader.h"
#include "articulation_interface.h"

using namespace physx;

class PxSimulation {
public:
  std::vector<PxRigidActor *> mRigidActors;
  PxPhysics *mPhysicsSDK = nullptr;
  PxFoundation *mFoundation = nullptr;
  PxCooking *mCooking = nullptr;
  PxScene *mScene = nullptr;
  PxReal mTimestep = 1.0f / 60.0f;
  IRenderer *mRenderer = nullptr;
  PxDefaultCpuDispatcher *mCpuDispatcher = nullptr;
  PxMaterial *mDefaultMaterial = nullptr;
  CollisionGroupManager collisionManager;

#ifdef _PVD
  PxPvd *mPvd = nullptr;
  PxPvdTransport *mTransport = nullptr;
#endif

  // std::map<PxRigidActor *, std::vector<physx_id_t>> mActor2Ids;
  std::map<physx_id_t, PxTransform> mRenderId2InitialPose;
  std::map<physx_id_t, PxRigidActor*> mRenderId2Parent;
  std::map<physx_id_t, IArticulationBase*> mRenderId2Articulation;

  std::vector<std::unique_ptr<struct PxArticulationWrapper>> mDynamicArticulationWrappers;

  // std::map<PxArticulationBase*, struct PxArticulationWrapper> mArticulation2Wrapper;

public:
  PxSimulation();
  ~PxSimulation();

  inline void setTimestep(PxReal step) { mTimestep = step; };
  inline PxReal getTimestep() const { return mTimestep; };

  void setRenderer(IRenderer *renderer);
  inline IRenderer *getRenderer() { return mRenderer; }

  std::unique_ptr<class PxActorBuilder> createActorBuilder();
  std::unique_ptr<class PxArticulationBuilder> createArticulationBuilder();

  /* advance physics by mTimestep */
  void step();

  /* Sync with renderer by calling UpdateRigidbody */
  void updateRenderer();
  PxRigidStatic *addGround(PxReal altitude, bool render = true, PxMaterial *material = nullptr);
};
