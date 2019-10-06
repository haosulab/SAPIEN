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
  std::map<physx_id_t, PxRigidActor *> mRenderId2Parent;
  std::map<physx_id_t, IArticulationBase *> mRenderId2Articulation;

  std::vector<std::unique_ptr<struct ArticulationWrapper>> mDynamicArticulationWrappers;
  std::vector<std::unique_ptr<class PxKinematicsArticulationWrapper>>
      mKinematicArticulationWrappers;
  std::vector<std::unique_ptr<class JointSystem>> mObjectArticulationWrappers;
  std::vector<std::unique_ptr<class ControllableArticulationWrapper>>
      mControllableArticulationWrapper;

public:
  Simulation();
  ~Simulation();

  inline void setTimestep(PxReal step) { mTimestep = step; };
  inline PxReal getTimestep() const { return mTimestep; };

  void setRenderer(Renderer::IPhysxRenderer *renderer);
  inline Renderer::IPhysxRenderer *getRenderer() { return mRenderer; }

  std::unique_ptr<class ActorBuilder> createActorBuilder();
  std::unique_ptr<class ArticulationBuilder> createArticulationBuilder();

  physx_id_t addMountedCamera(std::string const &name, PxRigidActor *actor,
                              PxTransform const &pose, uint32_t width, uint32_t height, float fovx,
                              float fovy, float near = 0.1, float far = 1000);

  /* advance physics by mTimestep */
  void step();

  /* Sync with renderer by calling UpdateRigidbody */
  void updateRenderer();
  PxRigidStatic *addGround(PxReal altitude, bool render = true, PxMaterial *material = nullptr);

  /* Create URDF loader from simulation */
  std::unique_ptr<URDF::URDFLoader> createURDFLoader();
};

} // namespace sapien
