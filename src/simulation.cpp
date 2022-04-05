#include <PxPhysicsAPI.h>
#include <extensions/PxExtensionsAPI.h>

#include "spdlog/sinks/stdout_color_sinks.h"
#include <spdlog/spdlog.h>

#include "filter_shader.h"
#include "simulation.h"

#include <easy/profiler.h>

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
  if (!spdlog::get("SAPIEN")) {
    auto logger = spdlog::stderr_color_mt("SAPIEN");
    setLogLevel("warn");
  }

  if (!profiler::isListening()) {
    profiler::startListen();
  }

  // TODO(fanbo): figure out what "track allocation" means in the PhysX doc
  mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, mErrorCallback);

  PxTolerancesScale toleranceScale;
  toleranceScale.length = toleranceLength;
  toleranceScale.speed = toleranceSpeed;

#ifdef _PVD
  spdlog::get("SAPIEN")->info("Connecting to PVD...");
  mTransport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 1000);
  mPvd = PxCreatePvd(*mFoundation);
  mPvd->connect(*mTransport, PxPvdInstrumentationFlag::eDEBUG);
  if (!mPvd->isConnected()) {
    spdlog::get("SAPIEN")->warn("Failed to connect to PVD");
    mPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, toleranceScale, false);
  } else {
    spdlog::get("SAPIEN")->info("PVD connected");
    mPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, toleranceScale, true, mPvd);
  }
#else

  mPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, toleranceScale);
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

  if (mRenderer) {
    mRenderer.reset();
  }
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

  return std::make_unique<SScene>(this->shared_from_this(), pxScene, config);
}

std::shared_ptr<SPhysicalMaterial> Simulation::createPhysicalMaterial(PxReal staticFriction,
                                                                      PxReal dynamicFriction,
                                                                      PxReal restitution) const {
  auto mat = mPhysicsSDK->createMaterial(staticFriction, dynamicFriction, restitution);
  mat->setFlag(PxMaterialFlag::eIMPROVED_PATCH_FRICTION, true);
  return std::make_shared<SPhysicalMaterial>(shared_from_this(), mat);
}

std::unique_ptr<SCollisionShape>
Simulation::createCollisionShape(PxGeometry const &geometry,
                                 std::shared_ptr<SPhysicalMaterial> material) {
  auto shape = mPhysicsSDK->createShape(geometry, *material->getPxMaterial(), true);
  auto result = std::make_unique<SCollisionShape>(shape);
  result->setPhysicalMaterial(material);
  return result;
}

void Simulation::setRenderer(std::shared_ptr<Renderer::IPxrRenderer> renderer) {
  if (mRenderer)
    spdlog::get("SAPIEN")->warn("Setting renderer more than once should be avoided.");
  mRenderer = renderer;
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
  } else if (level == "critical") {
    spdlog::get("SAPIEN")->set_level(spdlog::level::critical);
  } else if (level == "off") {
    spdlog::get("SAPIEN")->set_level(spdlog::level::off);
  } else {
    spdlog::error("Invalid log level \"{}\"", level);
  }
}

std::shared_ptr<Simulation> Simulation::getInstance(uint32_t nthread, PxReal toleranceLength,
                                                    PxReal toleranceSpeed) {
  static std::weak_ptr<Simulation> _instance;
  if (!_instance.expired()) {
    spdlog::get("SAPIEN")->warn("A second engine will share the same internal structures with "
                                "the first one. Arguments passed to constructor will be ignored.");
    return _instance.lock();
  }
  auto sim = std::make_shared<Simulation>(nthread, toleranceLength, toleranceSpeed);
  _instance = sim;
  return sim;
}

} // namespace sapien
