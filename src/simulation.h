/**
 * Sapien class for physical simulation.
 *
 * Notes:
 * 1. The physical simulation in Sapien is synchronous,
 * even if PhysX supports asynchronous simulation.
 * 2. Simulation should be maintained by std::shared_ptr.
 * Other global unique objects (like renderer) should also follow this rule.
 * The reason is to avoid being deleted before some local objects (like scene).
 *
 * References:
 * https://gameworksdocs.nvidia.com/PhysX/4.1/documentation/physxguide/Manual/Startup.html
 */

#pragma once

#include <memory>

#include <PxPhysicsAPI.h>

// TODO(jigu): check whether to replace with forward declaration
#include "mesh_manager.h"
#include "render_interface.h"
#include "sapien_material.h"
#include "sapien_scene.h"
#include "sapien_scene_config.h"
#include "sapien_shape.h"

namespace sapien {
using namespace physx;

class SapienErrorCallback : public PxErrorCallback {
  PxErrorCode::Enum mLastErrorCode = PxErrorCode::eNO_ERROR;

public:
  virtual void reportError(PxErrorCode::Enum code, const char *message, const char *file,
                           int line) override;
  PxErrorCode::Enum getLastErrorCode();
};

class Simulation : public std::enable_shared_from_this<Simulation> {
public:
  explicit Simulation(PxReal toleranceLength = 0.1f, PxReal toleranceSpeed = 0.2f);
  ~Simulation();

  std::unique_ptr<SScene> createScene(SceneConfig const &config = {});
  std::unique_ptr<SCollisionShape>
  createCollisionShape(PxGeometry const &geometry, std::shared_ptr<SPhysicalMaterial> material);

  inline std::shared_ptr<Renderer::IPxrRenderer> getRenderer() const { return mRenderer; }
  inline void setRenderer(std::shared_ptr<Renderer::IPxrRenderer> renderer) {
    mRenderer = renderer;
  }

  inline MeshManager &getMeshManager() { return mMeshManager; }
  void setLogLevel(std::string const &level);

#ifdef _PVD
  PxPvd *mPvd = nullptr;
  PxPvdTransport *mTransport = nullptr;
#endif

  PxPhysics *mPhysicsSDK = nullptr;
  PxCooking *mCooking = nullptr;

private:
  // PhysX objects
  PxFoundation *mFoundation = nullptr;
  PxDefaultCpuDispatcher *mCpuDispatcher = nullptr;
  SapienErrorCallback mErrorCallback;

  std::shared_ptr<Renderer::IPxrRenderer> mRenderer;

  MeshManager mMeshManager;
};

} // namespace sapien
