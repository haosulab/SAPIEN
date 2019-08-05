#include "simulation.h"
#include "actor_builder.h"
#include "articulation_builder.h"
#include <fstream>
#include <sstream>

#define PVD_HOST "10.0.0.123"

static PxDefaultErrorCallback gDefaultErrorCallback;
static PxDefaultAllocator gDefaultAllocatorCallback;
static PxSimulationFilterShader gDefaultFilterShader = PxDefaultSimulationFilterShader;

PxSimulation::PxSimulation() {
  mFoundation =
      PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
  // TODO: figure out the what "track allocation" means
  
#ifdef _PVD
  std::cerr << "Connecting to PVD..." << std::endl;
  mTransport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 100);
  mPvd = PxCreatePvd(*mFoundation);
  mPvd->connect(*mTransport, PxPvdInstrumentationFlag::eDEBUG);
  if (!mPvd->isConnected()) {
    std::cerr << "PVD connection failed." << std::endl;
  } else {
    mPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale(), true);
  }
  mPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale(), false, mPvd);
  // PxInitExtensions(*mPhysicsSDK, mPvd);
#else
  mPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale(), true);
#endif

  if (mPhysicsSDK == NULL) {
    std::cerr << "Error creating PhysX3 device." << std::endl;
    std::cerr << "Exiting..." << std::endl;
    exit(1);
  }

  mCooking =
      PxCreateCooking(PX_PHYSICS_VERSION, *mFoundation, PxCookingParams(PxTolerancesScale()));
  if (!mCooking) {
    std::cerr << "Error creating cooking." << std::endl;
    std::cerr << "Exiting..." << std::endl;
    exit(1);
  }

  if (!PxInitExtensions(*mPhysicsSDK, nullptr)) {
    std::cerr << "PxInitExtensions failed!" << std::endl;
    std::cerr << "Exiting..." << std::endl;
    exit(1);
  }

  // create scene
  PxSceneDesc sceneDesc(mPhysicsSDK->getTolerancesScale());
  sceneDesc.gravity = PxVec3(0.0f, 0.0f, -9.81f);

  // create dispatcher
  // TODO: check how GPU works here
  if (!sceneDesc.cpuDispatcher) {
    mCpuDispatcher = PxDefaultCpuDispatcherCreate(1);
    if (!mCpuDispatcher) {
      std::cerr << "PxDefaultCpuDispatcherCreate failed!" << std::endl;
      std::cerr << "Exiting..." << std::endl;
      exit(1);
    }
    sceneDesc.cpuDispatcher = mCpuDispatcher;
  }
  if (!sceneDesc.filterShader) {
    sceneDesc.filterShader = gDefaultFilterShader;
  }

  mScene = mPhysicsSDK->createScene(sceneDesc);
  if (!mScene) {
    std::cerr << "createScene failed!" << std::endl;
    std::cerr << "Exiting..." << std::endl;
    exit(1);
  }
  mScene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.f);
  mScene->setVisualizationParameter(PxVisualizationParameter::eACTOR_AXES, 2.f);

  mDefaultMaterial = mPhysicsSDK->createMaterial(0.5, 0.5, 0.5);
}

PxSimulation::~PxSimulation() {
  for (size_t i = 0; i < mRigidActors.size(); ++i) {
    mScene->removeActor(*mRigidActors[i]);
    mRigidActors[i]->release();
  }
  mDefaultMaterial->release();
  if (mCpuDispatcher) {
    mCpuDispatcher->release();
  }
  mRigidActors.clear();
  mScene->release();
  mCooking->release();
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

void PxSimulation::step() {
  mScene->simulate(mTimestep);
  while (!mScene->fetchResults()) {
    // TODO: do useful stuff here
  }
}

void PxSimulation::updateRenderer() {
  for (auto idParent : mRenderId2Parent) {
    auto pose = idParent.second->getGlobalPose() * mRenderId2InitialPose[idParent.first];

    mRenderer->updateRigidbody(idParent.first, idParent.second->getGlobalPose() *
                               mRenderId2InitialPose[idParent.first]);
  }
}

std::unique_ptr<PxActorBuilder> PxSimulation::createActorBuilder() {
  return std::make_unique<PxActorBuilder>(this);
}

std::unique_ptr<PxArticulationBuilder> PxSimulation::createArticulationBuilder() {
  return std::make_unique<PxArticulationBuilder>(this);
}
