#include "simulation.h"
#include "actor_builder.h"
#include "sapien_scene.h"
#include <cassert>
#include <fstream>
#include <memory>
#include <spdlog/spdlog.h>
#include <sstream>

namespace sapien {
static PxDefaultAllocator gDefaultAllocatorCallback;

void SapienErrorCallback::reportError(PxErrorCode::Enum code, const char *message,
                                      const char *file, int line) {
  mLastErrorCode = code;

#ifdef NDEBUG
  spdlog::critical("{}", message);
#else
  spdlog::critical("{}:{}: {}", file, line, message);
#endif
  // throw std::runtime_error("PhysX Error");
}

PxErrorCode::Enum SapienErrorCallback::getLastErrorCode() {
  auto code = mLastErrorCode;
  mLastErrorCode = PxErrorCode::eNO_ERROR;
  return code;
}

Simulation::Simulation(uint32_t nthread) : mThreadCount(nthread), mMeshManager(this) {
  mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, mErrorCallback);
  // FIXME: figure out the what "track allocation" means

#ifdef _PVD
  spdlog::info("Connecting to PVD...");
  mTransport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 1000);
  mPvd = PxCreatePvd(*mFoundation);
  mPvd->connect(*mTransport, PxPvdInstrumentationFlag::eDEBUG);
  if (!mPvd->isConnected()) {
    spdlog::warn("Failed to connect to PVD");
    mPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale(), true);
  } else {
    spdlog::info("PVD connected");
    mPhysicsSDK =
        PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale(), true, mPvd);
  }
#else
  mPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale(), true);
#endif

  if (!mPhysicsSDK) {
    spdlog::critical("Failed to create PhysX device");
    throw std::runtime_error("Simulation Creation Failed");
  }

  mCooking =
      PxCreateCooking(PX_PHYSICS_VERSION, *mFoundation, PxCookingParams(PxTolerancesScale()));
  if (!mCooking) {
    spdlog::critical("Failed to create PhysX Cooking");
    throw std::runtime_error("Simulation Creation Failed");
  }

  if (!PxInitExtensions(*mPhysicsSDK, nullptr)) {
    spdlog::critical("Failed to initialize PhysX Extensions");
    throw std::runtime_error("Simulation Creation Failed");
  }
  mDefaultMaterial = mPhysicsSDK->createMaterial(0.3, 0.3, 0.1);
}

Simulation::~Simulation() {
  mDefaultMaterial->release();
  if (mCpuDispatcher) {
    mCpuDispatcher->release();
  }
  mCooking->release();
  PxCloseExtensions();
  mPhysicsSDK->release();
#ifdef _PVD
  if (mPvd && mTransport) {
    mTransport->disconnect();
    mTransport->release();
    mPvd->release();
  }
#endif
  mFoundation->release();
}

void Simulation::setRenderer(Renderer::IPxrRenderer *renderer) { mRenderer = renderer; }

PxMaterial *Simulation::createPhysicalMaterial(PxReal staticFriction, PxReal dynamicFriction,
                                               PxReal restitution) const {
  return mPhysicsSDK->createMaterial(staticFriction, dynamicFriction, restitution);
}

std::unique_ptr<SScene> Simulation::createScene(PxVec3 gravity, PxSolverType::Enum solverType,
                                                PxSceneFlags sceneFlags) {

  PxSceneDesc sceneDesc(mPhysicsSDK->getTolerancesScale());
  sceneDesc.gravity = gravity;
  sceneDesc.filterShader = TypeAffinityIgnoreFilterShader;
  sceneDesc.solverType = solverType;
  sceneDesc.flags = sceneFlags;

  if (!mCpuDispatcher) {
    mCpuDispatcher = PxDefaultCpuDispatcherCreate(mThreadCount);
    if (!mCpuDispatcher) {
      spdlog::critical("Failed to create PhysX CPU dispatcher");
      throw std::runtime_error("Scene Creation Failed");
    }
  }
  sceneDesc.cpuDispatcher = mCpuDispatcher;

  PxScene *pxScene = mPhysicsSDK->createScene(sceneDesc);

  return std::make_unique<SScene>(this, pxScene);
}

} // namespace sapien
