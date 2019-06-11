#include "simulation.h"
#include <fstream>
#include <sstream>

static PxDefaultErrorCallback gDefaultErrorCallback;
static PxDefaultAllocator gDefaultAllocatorCallback;
static PxSimulationFilterShader gDefaultFilterShader =
    PxDefaultSimulationFilterShader;

PxSimulation::PxSimulation() {
  mFoundation = PxCreateFoundation(
      PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
  // TODO: figure out the what "track allocation" means
  mPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation,
                                PxTolerancesScale(), true);

  if (mPhysicsSDK == NULL) {
    std::cerr << "Error creating PhysX3 device." << std::endl;
    std::cerr << "Exiting..." << std::endl;
    exit(1);
  }

  mCooking = PxCreateCooking(PX_PHYSICS_VERSION, *mFoundation,
                             PxCookingParams(PxTolerancesScale()));
  if (!mCooking) {
    std::cerr << "Error creating cooking." << std::endl;
    std::cerr << "Exiting..." << std::endl;
    exit(1);
  }

  if (!PxInitExtensions(*mPhysicsSDK, nullptr)) {
    std::cerr << "PxInitExtensions failed!" << std::endl;
  }

  // create scene
  PxSceneDesc sceneDesc(mPhysicsSDK->getTolerancesScale());
  sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);

  // create dispatcher
  // TODO: check how GPU works here
  if (!sceneDesc.cpuDispatcher) {
    PxDefaultCpuDispatcher *mCpuDispatcher = PxDefaultCpuDispatcherCreate(1);
    if (!mCpuDispatcher)
      std::cerr << "PxDefaultCpuDispatcherCreate failed!" << std::endl;
    sceneDesc.cpuDispatcher = mCpuDispatcher;
  }
  if (!sceneDesc.filterShader)
    sceneDesc.filterShader = gDefaultFilterShader;

  mDefaultMaterial = mPhysicsSDK->createMaterial(0.5, 0.5, 0.5);
}

PxSimulation::~PxSimulation() {
  for (size_t i = 0; i < mRigidActors.size(); ++i) {
    mScene->removeActor(*mRigidActors[i]);
    mRigidActors[i]->release();
  }
  mRigidActors.clear();
  mScene->release();
  mPhysicsSDK->release();
  mFoundation->release();
}

void PxSimulation::step() {
  mScene->simulate(mTimestep);
  while (!mScene->fetchResults()) {
    // TODO: do useful stuff here
  }
}

PxActor* PxSimulation::addObj(const std::string &filename, bool isKinematic) {

  // load actor
  PxRigidDynamic *actor =
      mPhysicsSDK->createRigidDynamic(PxTransform(PxIdentity));
  actor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, isKinematic);
  PxTriangleMeshGeometry triGeom;
  PxConvexMesh* mesh = loadObjMesh(filename);
  PxShape *shape = PxRigidActorExt::createExclusiveShape(
      *actor, PxConvexMeshGeometry(mesh), *mDefaultMaterial);
  if (!shape) {
    std::cerr << "create shape failed!" << std::endl;
    return nullptr;
  }
  mScene->addActor(*actor);

  // update density
  PxRigidBodyExt::updateMassAndInertia(*actor, 1);

  // register actor
  uint64_t newId = actorCounter++;
  mActor2Id[actor] = newId;

  // add actor for rendering
  if (mRenderer) {
    mRenderer->addRigidbody(newId, filename);
  }

  return actor;
}

PxConvexMesh* PxSimulation::loadObjMesh(const std::string &filename) const {
  std::vector<PxVec3> vertices;

  std::ifstream f(
      "/home/fx/mobility_mesh/resources/46437-4/objs/original-1.obj");
  std::string line;

  std::string t;
  float a, b, c;
  while (std::getline(f, line)) {
    if (line[0] == '#') {
      continue;
    }
    std::istringstream iss(line);
    iss >> t;
    if (t == "v") {
      iss >> a >> b >> c;
      vertices.push_back({a, b, c});
    }
  }

  PxConvexMeshDesc convexDesc;
  convexDesc.points.count = vertices.size();
  convexDesc.points.stride = sizeof(PxVec3);
  convexDesc.points.data = vertices.data();
  convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

  PxDefaultMemoryOutputStream buf;
  PxConvexMeshCookingResult::Enum result;
  if (!mCooking->cookConvexMesh(convexDesc, buf, &result)) {
    std::cerr << "Unable to cook convex mesh." << std::endl;
    std::cerr << "Exiting..." << std::endl;
    exit(1);
  }
  PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
  PxConvexMesh *convexMesh = mPhysicsSDK->createConvexMesh(input);
  return convexMesh;
}
