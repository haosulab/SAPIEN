#include "simulation.h"
#include "actor_builder.h"
#include "articulation_builder.h"
#include "articulation_interface.h"
#include "my_filter_shader.h"
#include <cassert>
#include <fstream>
#include <sstream>
#include "kinematics_articulation_interface.h"

static PxDefaultErrorCallback gDefaultErrorCallback;
static PxDefaultAllocator gDefaultAllocatorCallback;
static PxSimulationFilterShader gDefaultFilterShader = PxDefaultSimulationFilterShader;

// class MyContactModification : public PxContactModifyCallback
//{
// public:
//    MyContactModification()
//            :PxContactModifyCallback() {
//
//    }
//    void onContactModify(PxContactModifyPair* const pairs, PxU32 count) {
//        // std::cout << "callback" << std::endl;
//        for (PxU32 i = 0; i < count; i++) {
//            // std::cout << "pair " << i << std::endl;
//            const PxRigidActor** actor = pairs[i].actor;
//            const PxShape** shapes = pairs[i].shape;
//            auto contacts = pairs[i].contacts;
//            for (PxU32 j = 0; j < contacts.size(); j++) {
//                // std::cout << contacts.getSeparation(j) << std::endl;
//            }
//        }
//    }
//};
//
// class MyContactCallback : public PxSimulationEventCallback {
// public:
//    MyContactCallback()
//            :PxSimulationEventCallback() {}
//    void onConstraintBreak(PxConstraintInfo* constraints, PxU32 count)	{ PX_UNUSED(constraints);
//    PX_UNUSED(count); } void onWake(PxActor** actors, PxU32 count)
//    { PX_UNUSED(actors); PX_UNUSED(count); }
//    void onSleep(PxActor** actors, PxU32 count)							{ PX_UNUSED(actors);
//    PX_UNUSED(count); } void onTrigger(PxTriggerPair* pairs, PxU32 count)
//    { PX_UNUSED(pairs); PX_UNUSED(count); } void onAdvance(const PxRigidBody*const*, const
//    PxTransform*, const PxU32) {} void onContact(const PxContactPairHeader& pairHeader, const
//    PxContactPair* pairs, PxU32 nbPairs) {
//        std::cerr << "force limit exceeded" << std::endl;
//    }
//};
// MyContactModification myCM;
// MyContactCallback myCC;

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
    mPhysicsSDK =
        PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale(), true, mPvd);
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
  // TODO: find a better way to sync data
  for (auto &wrapper : mDynamicArticulationWrappers) {
    wrapper->updateArticulation();
  }
  mScene->simulate(mTimestep);
  // TODO: find a better way to sync data
  while (!mScene->fetchResults(true)) {
    // TODO: do useful stuff here
  }
  for (auto &wrapper : mDynamicArticulationWrappers) {
    wrapper->updateCache();
  }
  for (auto & wrapper : mKinematicArticulationWrappers){
    wrapper->update(mTimestep);
  }
}

void PxSimulation::updateRenderer() {
  for (auto idParent : mRenderId2Parent) {
    auto pose = idParent.second->getGlobalPose() * mRenderId2InitialPose[idParent.first];

    mRenderer->updateRigidbody(idParent.first, pose);
  }
  for (auto &[id, pose] : mCameraId2InitialPose) {
    mRenderer->updateCamera(id, mMountedCamera2MountedActor[id]->getGlobalPose() * pose);
  }
}

std::unique_ptr<PxActorBuilder> PxSimulation::createActorBuilder() {
  return std::make_unique<PxActorBuilder>(this);
}

std::unique_ptr<PxArticulationBuilder> PxSimulation::createArticulationBuilder() {
  return std::make_unique<PxArticulationBuilder>(this);
}

void PxSimulation::setRenderer(Renderer::IPhysxRenderer *renderer) {
  mRenderer = renderer;
  mRenderer->bindQueryCallback([this](uint32_t unique_id) {
    Renderer::GuiInfo info = {};

    if (mRenderId2Parent.find(unique_id) == mRenderId2Parent.end()) {
      return info;
    }
    auto actor = this->mRenderId2Parent[unique_id];
    info.linkInfo.name = actor->getName();
    info.linkInfo.transform = actor->getGlobalPose();
    if (mRenderId2Articulation.find(unique_id) == mRenderId2Articulation.end()) {
      return info;
    }
    IArticulationBase *articulation = mRenderId2Articulation[unique_id];

    std::vector<std::string> singleDofName;
    auto totalDofs = articulation->dof();
    auto dofs = articulation->get_joint_dofs();
    auto names = articulation->get_joint_names();
    auto limits = articulation->get_joint_limits();
    auto values = articulation->get_qpos();
    for (uint32_t i = 0; i < dofs.size(); ++i) {
      for (uint32_t j = 0; j < dofs[i]; ++j) {
        singleDofName.push_back(names[i]);
      }
    }

    assert(singleDofName.size() == totalDofs);
    for (uint32_t i = 0; i < totalDofs; ++i) {
      Renderer::JointGuiInfo jointInfo;
      jointInfo.name = singleDofName[i];
      auto [l1, l2] = limits[i];
      jointInfo.limits = {l1, l2};
      jointInfo.value = values[i];
      info.articulationInfo.jointInfo.push_back(jointInfo);
    }
    return info;
  });

  mRenderer->bindSyncCallback([this](uint32_t unique_id, const Renderer::GuiInfo &info) {
    if (this->mRenderId2Parent.find(unique_id) == this->mRenderId2Parent.end()) {
      throw std::runtime_error("queried id is not an actor!");
    }
    if (this->mRenderId2Articulation.find(unique_id) == this->mRenderId2Articulation.end()) {
      throw std::runtime_error("queried id is not an articulation!");
    }
    auto articulation = mRenderId2Articulation[unique_id];

    std::vector<float> jointValues;
    for (auto &info : info.articulationInfo.jointInfo) {
      jointValues.push_back(info.value);
    }
    articulation->set_qpos(jointValues);
  });
}

PxRigidStatic *PxSimulation::addGround(PxReal altitude, bool render, PxMaterial *material) {
  material = material ? material : mDefaultMaterial;
  auto ground = PxCreatePlane(*mPhysicsSDK, PxPlane(0.f, 0.f, 1.f, -altitude), *material);
  ground->setName("Ground");
  mScene->addActor(*ground);

  if (render) {
    physx_id_t newId = IDGenerator::instance()->next();
    mRenderer->addRigidbody(newId, PxGeometryType::ePLANE, {10, 10, 10});
    mRenderId2InitialPose[newId] = PxTransform({0, 0, altitude}, PxIdentity);
    mRenderId2Parent[newId] = ground;
  }
  return ground;
}
