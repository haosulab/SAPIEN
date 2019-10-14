#include "simulation.h"
#include "actor_builder.h"
#include "articulation_builder.h"
#include "articulation_interface.h"
#include "controllable_articulation_wrapper.h"
#include "kinematics_articulation_wrapper.h"
#include <cassert>
#include <sstream>

namespace sapien {
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
class MyContactCallback : public PxSimulationEventCallback {
public:
  MyContactCallback() : PxSimulationEventCallback() {}
  void onConstraintBreak(PxConstraintInfo *constraints, PxU32 count) {
    PX_UNUSED(constraints);
    PX_UNUSED(count);
  }
  void onWake(PxActor **actors, PxU32 count) {
    PX_UNUSED(actors);
    PX_UNUSED(count);
  }
  void onSleep(PxActor **actors, PxU32 count) {
    PX_UNUSED(actors);
    PX_UNUSED(count);
  }
  void onTrigger(PxTriggerPair *pairs, PxU32 count) {
    PX_UNUSED(pairs);
    PX_UNUSED(count);
  }
  void onAdvance(const PxRigidBody *const *, const PxTransform *, const PxU32) {}
  void onContact(const PxContactPairHeader &pairHeader, const PxContactPair *pairs,
                 PxU32 nbPairs) {
    // if (std::string(pairHeader.actors[0]->getName()) != "Ground" &&
    //     std::string(pairHeader.actors[1]->getName()) != "Ground") {
    //   return;
    // }
    // std::cout << "Names " << pairHeader.actors[0]->getName() << " "
    //           << pairHeader.actors[1]->getName() << std::endl;

    // for (PxU32 i = 0; i < nbPairs; i++) {
    //   const PxU32 bufferSize = 64;
    //   PxContactPairPoint contacts[bufferSize];
    //   PxU32 nbContacts = pairs[i].extractContacts(contacts, bufferSize);
    //   for (PxU32 j = 0; j < nbContacts; j++) {
    //     PxVec3 impulse = contacts[j].impulse;
    //     printf("Impulse %f %f %f\n", impulse.x, impulse.y, impulse.z);
    //   }
    // }
  }
};
// MyContactModification myCM;
MyContactCallback myCC;

Simulation::Simulation() {
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
  mScene->setSimulationEventCallback(&myCC);

  mDefaultMaterial = mPhysicsSDK->createMaterial(0.5, 0.49, 0.01);
}

Simulation::~Simulation() {
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

void Simulation::step() {
  for (auto &wrapper : mKinematicArticulationWrappers) {
    wrapper->update(mTimestep);
  }
  mScene->simulate(mTimestep);
  while (!mScene->fetchResults(true)) {
    // TODO: do useful stuff here
  }
  for (auto &wrapper : mDynamicArticulationWrappers) {
    wrapper->update();
  }
  for (auto &wrapper : mControllableArticulationWrapper) {
    wrapper->update(mTimestep);
  }
}

void Simulation::updateRenderer() {
  for (auto idParent : mRenderId2Actor) {
    auto pose = idParent.second->getGlobalPose() * mRenderId2InitialPose[idParent.first];

    mRenderer->updateRigidbody(idParent.first, pose);
  }
  for (auto &[id, pose] : mCameraId2InitialPose) {
    mRenderer->updateCamera(id, mMountedCamera2MountedActor[id]->getGlobalPose() * pose);
  }
}

std::unique_ptr<ActorBuilder> Simulation::createActorBuilder() {
  return std::make_unique<ActorBuilder>(this);
}

std::unique_ptr<ArticulationBuilder> Simulation::createArticulationBuilder() {
  return std::make_unique<ArticulationBuilder>(this);
}

void Simulation::setRenderer(Renderer::IPhysxRenderer *renderer) {
  mRenderer = renderer;
  mRenderer->bindQueryCallback([this](uint32_t seg_id) {
    Renderer::GuiInfo info = {};

    if (mLinkId2Actor.find(seg_id) == mLinkId2Actor.end()) {
      return info;
    }
    auto actor = this->mLinkId2Actor[seg_id];
    info.linkInfo.name = actor->getName();
    info.linkInfo.transform = actor->getGlobalPose();
    if (mLinkId2Articulation.find(seg_id) == mLinkId2Articulation.end()) {
      return info;
    }
    IArticulationBase *articulation = mLinkId2Articulation[seg_id];

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
      l1 = std::max(l1, -10.f);
      l2 = std::min(l2, 10.f);
      jointInfo.limits = {l1, l2};
      jointInfo.value = values[i];
      info.articulationInfo.jointInfo.push_back(jointInfo);
    }
    return info;
  });

  mRenderer->bindSyncCallback([this](uint32_t seg_id, const Renderer::GuiInfo &info) {
    if (this->mLinkId2Actor.find(seg_id) == this->mLinkId2Actor.end()) {
      throw std::runtime_error("queried id is not an actor!");
    }
    if (this->mLinkId2Articulation.find(seg_id) == this->mLinkId2Articulation.end()) {
      throw std::runtime_error("queried id is not an articulation!");
    }
    auto articulation = mLinkId2Articulation[seg_id];

    std::vector<float> jointValues;
    for (auto &info : info.articulationInfo.jointInfo) {
      jointValues.push_back(info.value);
    }
    articulation->set_qpos(jointValues);
  });
}

PxRigidStatic *Simulation::addGround(PxReal altitude, bool render, PxMaterial *material) {
  material = material ? material : mDefaultMaterial;
  auto ground = PxCreatePlane(*mPhysicsSDK, PxPlane(0.f, 0.f, 1.f, -altitude), *material);
  ground->setName("Ground");
  mScene->addActor(*ground);

  if (render) {
    physx_id_t newId = IDGenerator::RenderId()->next();
    mRenderer->addRigidbody(newId, PxGeometryType::ePLANE, {10, 10, 10}, {1, 1, 1});
    mRenderId2InitialPose[newId] = PxTransform({0, 0, altitude}, PxIdentity);
    mRenderId2Actor[newId] = ground;
  }
  return ground;
}

physx_id_t Simulation::addMountedCamera(std::string const &name, PxRigidActor *actor,
                                        PxTransform const &pose, uint32_t width, uint32_t height,
                                        float fovx, float fovy, float near, float far) {
  physx_id_t cameraId = IDGenerator::RenderId()->next();
  const PxVec3 up = {0, 0, 1};
  const PxVec3 forward = {1, 0, 0};
  const PxMat33 rot(forward.cross(up), up, -forward);

  mMountedCamera2MountedActor[cameraId] = actor;
  mCameraId2InitialPose[cameraId] = pose * PxTransform(PxVec3(0), PxQuat(rot));

  mRenderer->addCamera(cameraId, name, width, height, fovx, fovy, near, far);

  return cameraId;
}
std::unique_ptr<URDF::URDFLoader> Simulation::createURDFLoader() {
  auto loader = std::make_unique<URDF::URDFLoader>(*this);
  return loader;
}

PxMaterial *Simulation::createPhysicalMaterial(PxReal staticFriction, PxReal dynamicFriction,
                                               PxReal restitution) const {
  return mPhysicsSDK->createMaterial(staticFriction, dynamicFriction, restitution);
}
class ControllableArticulationWrapper *
Simulation::createControllableArticulationWrapper(class IArticulationDrivable *baseWrapper) {
  auto wrapper = std::make_unique<ControllableArticulationWrapper>(baseWrapper);
  wrapper->updateTimeStep(mTimestep);
  auto wrapperPtr = wrapper.get();
  mControllableArticulationWrapper.push_back(wrapper);
  return wrapperPtr;
}
} // namespace sapien
