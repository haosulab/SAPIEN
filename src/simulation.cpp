#include "simulation.h"
#include "actor_builder.h"
#include "articulation_builder.h"
#include <fstream>
#include <sstream>

static PxDefaultErrorCallback gDefaultErrorCallback;
static PxDefaultAllocator gDefaultAllocatorCallback;
static PxSimulationFilterShader gDefaultFilterShader = PxDefaultSimulationFilterShader;

PxSimulation::PxSimulation() {
  mFoundation =
      PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
  // TODO: figure out the what "track allocation" means
  
#ifdef _PVD
  std::cerr << "Connecting to PVD..." << std::endl;
  mTransport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 1000);
  mPvd = PxCreatePvd(*mFoundation);
  mPvd->connect(*mTransport, PxPvdInstrumentationFlag::eDEBUG);
  if (!mPvd->isConnected()) {
    std::cerr << "PVD connection failed." << std::endl;
    mPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale(), true);
  } else {
    std::cout << "PVD connected." << std::endl;
    mPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale(), true, mPvd);
  }
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
  sceneDesc.filterShader = StandardFilterShader;

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
  while (!mScene->fetchResults(true)) {
    // TODO: do useful stuff here
  }
}

void PxSimulation::updateRenderer() {
  for (auto idParent : mRenderId2Parent) {
    auto pose = idParent.second->getGlobalPose() * mRenderId2InitialPose[idParent.first];

    mRenderer->updateRigidbody(idParent.first, pose);
  }
}

std::unique_ptr<PxActorBuilder> PxSimulation::createActorBuilder() {
  return std::make_unique<PxActorBuilder>(this);
}

std::unique_ptr<PxArticulationBuilder> PxSimulation::createArticulationBuilder() {
  return std::make_unique<PxArticulationBuilder>(this);
}

void PxSimulation::setRenderer(IRenderer *renderer) {
  mRenderer = renderer;
  mRenderer->bindQueryCallback(
      [this](uint32_t unique_id) {
        GuiInfo info = {};
        if (this->mRenderId2Parent.find(unique_id) == this->mRenderId2Parent.end()) {
          std::cerr << "queried id not found!" << std::endl;
          return info;
        }
        auto actor = this->mRenderId2Parent[unique_id];
        info.linkInfo.name = actor->getName();
        info.linkInfo.transform = actor->getGlobalPose();

        if (actor->getType() != PxActorType::eARTICULATION_LINK) {
          return info;
        }
        auto link = static_cast<PxArticulationLink *>(actor);
        // TODO: handle other types of articulation
        if (link) {
          auto it = mArticulation2Wrapper.find(&link->getArticulation());
          if (it == mArticulation2Wrapper.end()) {
            return info;
          }
          auto & wrapper = it->second;
          info.articulationInfo.name = wrapper.articulation->getName();
          wrapper.updateCache();
          for (uint32_t i = 0; i < wrapper.jointSummary.size(); ++i) {
            JointGuiInfo jointInfo;
            jointInfo.name = wrapper.jointSummary[i].name;
            jointInfo.limits = { wrapper.jointSummary[i].limitLow, wrapper.jointSummary[i].limitHigh };
            jointInfo.value = wrapper.cache->jointPosition[i];
            info.articulationInfo.jointInfo.push_back(jointInfo);
          }
        }
        return info;
      });
  mRenderer->bindSyncCallback(
      [this](uint32_t unique_id, const GuiInfo &info) {
        if (this->mRenderId2Parent.find(unique_id) == this->mRenderId2Parent.end()) {
          std::cerr << "queried id not found!" << std::endl;
          return;
        }

        auto actor = this->mRenderId2Parent[unique_id];
        if (actor->getType() != PxActorType::eARTICULATION_LINK) {
          return;
        }
        auto link = static_cast<PxArticulationLink *>(actor);
        // TODO: handle other types of articulation
        if (link) {
          auto it = mArticulation2Wrapper.find(&link->getArticulation());
          if (it == mArticulation2Wrapper.end()) {
            return;
          }
          auto &wrapper = it->second;
          if (wrapper.dof() != info.articulationInfo.jointInfo.size()) {
            std::cerr << "synced dof does not match articulation dof, something is wrong!" << std::endl;
            return;
          }
          std::vector<float> jointValues;
          for (auto &info : info.articulationInfo.jointInfo) {
            jointValues.push_back(info.value);
          }
          wrapper.set_qpos_unchecked(jointValues);
          wrapper.updateArticulation();
        }
  });
}
