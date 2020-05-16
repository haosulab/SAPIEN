#pragma once
#include "filter_shader.h"
#include "id_generator.h"
#include "mesh_manager.h"
#include "render_interface.h"
#include "sapien_scene_config.h"
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

private:
  uint32_t mThreadCount = 0;
  Renderer::IPxrRenderer *mRenderer = nullptr;
  PxDefaultCpuDispatcher *mCpuDispatcher = nullptr;

private:
  MeshManager mMeshManager;

public:
  inline MeshManager &getMeshManager() { return mMeshManager; }

public:
  std::unique_ptr<SScene> createScene(SceneConfig const &config = {});

#ifdef _PVD
  PxPvd *mPvd = nullptr;
  PxPvdTransport *mTransport = nullptr;
#endif

public:
  explicit Simulation(uint32_t nthread = 0, PxReal toleranceLength = 0.1f,
                      PxReal toleranceSpeed = 0.2f);
  ~Simulation();

  void setRenderer(Renderer::IPxrRenderer *renderer);
  inline Renderer::IPxrRenderer *getRenderer() { return mRenderer; }
  void setLogLevel(std::string const &level);

  PxMaterial *createPhysicalMaterial(PxReal staticFriction, PxReal dynamicFriction,
                                     PxReal restitution) const;
};

} // namespace sapien
