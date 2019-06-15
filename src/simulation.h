#pragma once
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
#include <vector>
#include <memory>
#include "id_generator.h"


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

  std::map<PxRigidActor *, std::vector<physx_id_t>> mActor2Ids;

public:
  PxSimulation();
  ~PxSimulation();

  inline void setTimestep(PxReal step) { mTimestep = step; };
  inline PxReal getTimestep() const { return mTimestep; };

  inline void setRenderer(IRenderer *renderer) { mRenderer = renderer; }
  inline IRenderer *getRenderer() { return mRenderer; }

  // Helper for add Obj
  // TODO: obj file caching
  // PxConvexMesh *loadObjMesh(const std::string &filename) const;

  // TODO: density, mass, etc
  /* Add obj file to scene, sync with renderer */
  // PxActor *addObj(const std::string &filename, bool isKinematic = false,
  //                 PxVec3 p = PxVec3(0.f, 0.f, 0.f), PxQuat q = PxIdentity);
  // PxActor *addGroundPlane();

  std::unique_ptr<class PxActorBuilder> createActorBuilder();

  /* advance physics by mTimestep */
  void step();

  /* Sync with renderer by calling UpdateRigidbody */
  void updateRenderer();
};
