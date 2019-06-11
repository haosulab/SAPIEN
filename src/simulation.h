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

using namespace physx;

class PxSimulation {
  std::vector<PxRigidActor *> mRigidActors;
  PxPhysics *mPhysicsSDK = nullptr;
  PxFoundation *mFoundation = nullptr;
  PxCooking *mCooking = nullptr;
  PxScene *mScene = nullptr;
  PxReal mTimestep = 1.0f / 600.0f;
  IRenderer *mRenderer = nullptr;

  PxMaterial *mDefaultMaterial = nullptr;

private:
  uint64_t actorCounter = 0;
  std::map<PxRigidActor *, uint64_t> mActor2Id;

public:
  PxSimulation();
  ~PxSimulation();

  inline void setTimestep(PxReal step) { mTimestep = step; };
  inline PxReal getTimestep() const { return mTimestep; };

  inline void setRenderer(IRenderer *renderer) { mRenderer = renderer; }
  inline IRenderer *getRenderer() { return mRenderer; }

  PxActor *addObj(const std::string &filename, bool isKinematic = false);

  PxConvexMesh *loadObjMesh(const std::string &filename) const;

  void step();
};
