#include "simulation.h"
#include "actor_builder.h"
#include "sapien_scene.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include <cassert>
#include <fstream>
#include <memory>
#include <spdlog/spdlog.h>
#include <sstream>

#ifdef _PROFILE
#include <easy/profiler.h>
#endif

namespace sapien {
static PxDefaultAllocator gDefaultAllocatorCallback;

void SapienErrorCallback::reportError(PxErrorCode::Enum code, const char *message,
                                      const char *file, int line) {
  mLastErrorCode = code;

#ifdef NDEBUG
  spdlog::get("SAPIEN")->critical("{}", message);
#else
  spdlog::get("SAPIEN")->critical("{}:{}: {}", file, line, message);
#endif
  // throw std::runtime_error("PhysX Error");
}

PxErrorCode::Enum SapienErrorCallback::getLastErrorCode() {
  auto code = mLastErrorCode;
  mLastErrorCode = PxErrorCode::eNO_ERROR;
  return code;
}

Simulation::Simulation(uint32_t nthread, PxReal toleranceLength, PxReal toleranceSpeed)
    : mThreadCount(nthread), mMeshManager(this) {
  auto logger = spdlog::stderr_color_mt("SAPIEN");
  setLogLevel("warn");

#ifdef _PROFILE
  profiler::startListen();
  spdlog::get("SAPIEN")->info("Profiling enabled");
#endif

  mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, mErrorCallback);
  // FIXME: figure out the what "track allocation" means

#ifdef _PVD
  spdlog::get("SAPIEN")->info("Connecting to PVD...");
  mTransport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 1000);
  mPvd = PxCreatePvd(*mFoundation);
  mPvd->connect(*mTransport, PxPvdInstrumentationFlag::eDEBUG);
  if (!mPvd->isConnected()) {
    spdlog::get("SAPIEN")->warn("Failed to connect to PVD");
    mPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale(), true);
  } else {
    spdlog::get("SAPIEN")->info("PVD connected");
    mPhysicsSDK =
        PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale(), true, mPvd);
  }
#else
  PxTolerancesScale toleranceScale;
  toleranceScale.length = toleranceLength;
  toleranceScale.speed = toleranceSpeed;

  mPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, toleranceScale, true);
#endif

  if (!mPhysicsSDK) {
    spdlog::get("SAPIEN")->critical("Failed to create PhysX device");
    throw std::runtime_error("Simulation Creation Failed");
  }

  mCooking = PxCreateCooking(PX_PHYSICS_VERSION, *mFoundation, PxCookingParams(toleranceScale));
  if (!mCooking) {
    spdlog::get("SAPIEN")->critical("Failed to create PhysX Cooking");
    throw std::runtime_error("Simulation Creation Failed");
  }

  if (!PxInitExtensions(*mPhysicsSDK, nullptr)) {
    spdlog::get("SAPIEN")->critical("Failed to initialize PhysX Extensions");
    throw std::runtime_error("Simulation Creation Failed");
  }
}

Simulation::~Simulation() {
  // mDefaultMaterial->release();
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

std::unique_ptr<SScene> Simulation::createScene(SceneConfig const &config) {

  PxSceneDesc sceneDesc(mPhysicsSDK->getTolerancesScale());
  sceneDesc.gravity = PxVec3({config.gravity.x(), config.gravity.y(), config.gravity.z()});
  sceneDesc.filterShader = TypeAffinityIgnoreFilterShader;
  sceneDesc.solverType = config.enableTGS ? PxSolverType::eTGS : PxSolverType::ePGS;
  sceneDesc.bounceThresholdVelocity = config.bounceThreshold;
  PxSceneFlags sceneFlags;
  if (config.enableEnhancedDeterminism) {
    sceneFlags |= PxSceneFlag::eENABLE_ENHANCED_DETERMINISM;
  }
  if (config.enablePCM) {
    sceneFlags |= PxSceneFlag::eENABLE_PCM;
  }
  if (config.enableCCD) {
    sceneFlags |= PxSceneFlag::eENABLE_CCD;
  }
  if (config.enableFrictionEveryIteration) {
    sceneFlags |= PxSceneFlag::eENABLE_FRICTION_EVERY_ITERATION;
  }
  if (config.enableAdaptiveForce) {
    sceneFlags |= PxSceneFlag::eADAPTIVE_FORCE;
  }

  sceneDesc.flags = sceneFlags;

  if (!mCpuDispatcher) {
    mCpuDispatcher = PxDefaultCpuDispatcherCreate(mThreadCount);
    if (!mCpuDispatcher) {
      spdlog::get("SAPIEN")->critical("Failed to create PhysX CPU dispatcher");
      throw std::runtime_error("Scene Creation Failed");
    }
  }
  sceneDesc.cpuDispatcher = mCpuDispatcher;

  PxScene *pxScene = mPhysicsSDK->createScene(sceneDesc);

  return std::make_unique<SScene>(this, pxScene, config);
}

void Simulation::setLogLevel(std::string const &level) {
  if (level == "debug") {
    spdlog::get("SAPIEN")->set_level(spdlog::level::debug);
  } else if (level == "info") {
    spdlog::get("SAPIEN")->set_level(spdlog::level::info);
  } else if (level == "warn" || level == "warning") {
    spdlog::get("SAPIEN")->set_level(spdlog::level::warn);
  } else if (level == "err" || level == "error") {
    spdlog::get("SAPIEN")->set_level(spdlog::level::err);
  } else {
    spdlog::error("Invalid log level \"{}\"", level);
  }
}

} // namespace sapien
