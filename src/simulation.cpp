#include <PxPhysicsAPI.h>
#include <extensions/PxExtensionsAPI.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include "sapien/filter_shader.h"
#include "sapien/simulation.h"

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

#ifdef SAPIEN_PROFILE
  if (!profiler::isListening()) {
    profiler::startListen();
  }
#endif

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

  if (!PxInitExtensions(*mPhysicsSDK, nullptr)) {
    spdlog::get("SAPIEN")->critical("Failed to initialize PhysX Extensions");
    throw std::runtime_error("Simulation Creation Failed");
  }
}

Simulation::~Simulation() {
  if (mCpuDispatcher) {
    mCpuDispatcher->release();
  }
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
  return std::make_unique<SScene>(this->shared_from_this(), config);
}

std::shared_ptr<SPhysicalMaterial> Simulation::createPhysicalMaterial(PxReal staticFriction,
                                                                      PxReal dynamicFriction,
                                                                      PxReal restitution) {
  physx_id_t materialId = mPhysicalMaterialIdGenerator.next();

  auto mat = mPhysicsSDK->createMaterial(staticFriction, dynamicFriction, restitution);
  mat->setFlag(PxMaterialFlag::eIMPROVED_PATCH_FRICTION, true);
  return std::make_shared<SPhysicalMaterial>(shared_from_this(), mat, materialId);
}

std::unique_ptr<SCollisionShape>
Simulation::createCollisionShape(PxGeometry const &geometry,
                                 std::shared_ptr<SPhysicalMaterial> material) {
  auto shape = mPhysicsSDK->createShape(geometry, *material->getPxMaterial(), true);
  auto result = std::make_unique<SCollisionShape>(shape);
  result->setPhysicalMaterial(material);
  return result;
}

std::shared_ptr<SNonconvexMeshGeometry>
Simulation::createMeshGeometry(std::vector<physx::PxReal> vertices, std::vector<uint32_t> indices,
                               physx::PxVec3 scale, physx::PxQuat rotation) {
  if (vertices.size() % 3 != 0 || indices.size() % 3 != 0) {
    throw std::runtime_error("failed to create geometry: invalid vertex or index size");
  }
  for (uint32_t i : indices) {
    if (i >= vertices.size()) {
      throw std::runtime_error("failed to create geometry: some index >= vertex count.");
    }
  }
  auto geometry = std::make_shared<SNonconvexMeshGeometry>();
  geometry->vertices = vertices;
  geometry->indices = indices;
  geometry->scale = scale;
  geometry->rotation = rotation;
  return geometry;
}

void Simulation::setRenderer(std::shared_ptr<Renderer::IRenderer> renderer) {
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

static std::weak_ptr<Simulation> _instance{};
std::shared_ptr<Simulation> Simulation::getInstance(uint32_t nthread, PxReal toleranceLength,
                                                    PxReal toleranceSpeed) {
  if (auto sim = _instance.lock()) {
    if (sim) {
      auto scale = sim->mPhysicsSDK->getTolerancesScale();
      PxReal dl = scale.length - toleranceLength;
      PxReal ds = scale.speed - toleranceSpeed;
      if (abs(dl) > 1e-6f || abs(ds) > 1e-6f) {
        spdlog::get("SAPIEN")->warn("Creating multiple engines with different parameters is not "
                                    "allowed. Parameters will be ignored.");
      }
      return sim;
    }
  }

  auto sim = std::make_shared<Simulation>(nthread, toleranceLength, toleranceSpeed);
  _instance = sim;
  return sim;
}

} // namespace sapien
