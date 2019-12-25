#include "simulation.h"
#include "actor_builder.h"
#include "articulation_builder.h"
#include "articulation_interface.h"
#include "controllable_articulation_wrapper.h"
#include "kinematics_articulation_wrapper.h"
#include <cassert>
#include <fstream>
#include <spdlog/spdlog.h>
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

Simulation::Simulation() : mMeshManager(this) {
  mFoundation =
      PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
  // TODO: figure out the what "track allocation" means

#ifdef _PVD
  spdlog::info("Connecting to PVD...");
  mTransport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 1000);
  mPvd = PxCreatePvd(*mFoundation);
  mPvd->connect(*mTransport, PxPvdInstrumentationFlag::eDEBUG);
  if (!mPvd->isConnected()) {
    spdlog::warn("Cannot connect to PVD");
    mPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale(), true);
  } else {
    spdlog::info("PVD connected");
    mPhysicsSDK =
        PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale(), true, mPvd);
  }
#else
  mPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale(), true);
#endif

  if (mPhysicsSDK == NULL) {
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

  // create scene
  PxSceneDesc sceneDesc(mPhysicsSDK->getTolerancesScale());
  sceneDesc.gravity = PxVec3(0.0f, 0.0f, -9.81f);
  sceneDesc.filterShader = StandardFilterShader;
  sceneDesc.solverType = PxSolverType::eTGS;
  sceneDesc.flags |= PxSceneFlag::eENABLE_CCD;

  // create dispatcher
  // TODO: check how GPU works here
  if (!sceneDesc.cpuDispatcher) {
    mCpuDispatcher = PxDefaultCpuDispatcherCreate(1);
    if (!mCpuDispatcher) {
      spdlog::critical("Failed to create PhysX CPU dispatcher");
      throw std::runtime_error("Scene Creation Failed");
    }
    sceneDesc.cpuDispatcher = mCpuDispatcher;
  }
  if (!sceneDesc.filterShader) {
    sceneDesc.filterShader = gDefaultFilterShader;
  }

  mScene = mPhysicsSDK->createScene(sceneDesc);
  if (!mScene) {
    spdlog::critical("Failed to create PhysX Scene");
    throw std::runtime_error("Scene Creation Failed");
  }
  mScene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.f);
  mScene->setVisualizationParameter(PxVisualizationParameter::eACTOR_AXES, 2.f);
  mScene->setSimulationEventCallback(&myCC);

  mDefaultMaterial = mPhysicsSDK->createMaterial(5, 5, 0.01);
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
  for (auto &callBack : mStepCallBacks) {
    callBack(*this);
  }
}

void Simulation::updateRenderer() {
  if (!mRenderer)
    return;
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
    auto limits = articulation->get_qlimits();
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

  mRenderer->bindSaveActionCallback([this](uint32_t index, uint32_t action) {
    switch (action) {
    case 0:
      loadSave(index);
      break;
    case 1:
      deleteSave(index);
    default:
      break;
    }

    std::vector<std::string> names;
    for (auto &s : simulationSaves) {
      names.push_back(s.name);
    }
    mRenderer->setSaveNames(names);
  });

  mRenderer->bindSaveCallback([this](uint32_t index, const std::string &name) {
    if (index < simulationSaves.size()) {
      simulationSaves[index].name = name;
    } else {
      appendSaves(name);
    }
    std::vector<std::string> names;
    for (auto &s : simulationSaves) {
      names.push_back(s.name);
    }
    mRenderer->setSaveNames(names);
  });
}

PxRigidStatic *Simulation::addGround(PxReal altitude, bool render, PxMaterial *material) {
  material = material ? material : mDefaultMaterial;
  auto ground = PxCreatePlane(*mPhysicsSDK, PxPlane(0.f, 0.f, 1.f, -altitude), *material);
  ground->setName("Ground");
  mScene->addActor(*ground);

  if (render && mRenderer) {
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
  auto wrapper = std::make_unique<ControllableArticulationWrapper>(baseWrapper, this);
  wrapper->updateTimeStep(mTimestep);
  auto wrapperPtr = wrapper.get();
  mControllableArticulationWrapper.push_back(std::move(wrapper));
  return wrapperPtr;
}
std::vector<PxReal> Simulation::dump() {
  std::vector<PxReal> data;
  size_t size = 0;
  for (auto const &w : mDynamicArticulationWrappers) {
    auto pose = w->links[0]->getGlobalPose();
    std::vector<PxReal> poseVec = {pose.p.x, pose.p.y, pose.p.z, pose.q.w,
                                   pose.q.x, pose.q.y, pose.q.z};
    std::vector<PxReal> qpos(w->get_qpos());
    std::vector<PxReal> qvel(w->get_qvel());
    std::vector<PxReal> qacc(w->get_qacc());
    std::vector<PxReal> qf(w->get_qf());
    auto dof = w->dof();
    size += 4 * dof + 7;
    data.reserve(size);
    data.insert(data.end(), poseVec.begin(), poseVec.end());
    data.insert(data.end(), qpos.begin(), qpos.end());
    data.insert(data.end(), qvel.begin(), qvel.end());
    data.insert(data.end(), qacc.begin(), qacc.end());
    data.insert(data.end(), qf.begin(), qf.end());
  }
  return data;
}
void Simulation::pack(const std::vector<PxReal> &data) {
  auto begin = data.begin();
  for (auto const &w : mDynamicArticulationWrappers) {
    auto dof = w->dof();
    auto poseVec = std::vector<PxReal>(begin, begin + 7);
    PxTransform pose({poseVec[0], poseVec[1], poseVec[2]},
                     {poseVec[4], poseVec[5], poseVec[6], poseVec[3]});
    w->articulation->teleportRootLink(pose, true);
    begin += 7;
    w->set_qpos(std::vector<PxReal>(begin, begin + dof));
    begin += dof;
    w->set_qvel(std::vector<PxReal>(begin, begin + dof));
    begin += dof;
    w->set_qacc(std::vector<PxReal>(begin, begin + dof));
    begin += dof;
    w->set_qf(std::vector<PxReal>(begin, begin + dof));
    begin += dof;
    for (auto &body : w->links) {
      body->setForceAndTorque({0, 0, 0}, {0, 0, 0});
    }
  }
  assert(begin == data.end());
  clearCache();
}
void Simulation::clearCache() {
  for (auto &i : mControllableArticulationWrapper) {
    i->clearCache();
  }
}

void Simulation::writeSavesToDisk(const std::string &filename) {
  auto of = std::ofstream(filename, std::ios::out | std::ios::binary);
  int size = static_cast<int>(simulationSaves.size());
  of << size;

  for (auto &cache : simulationSaves) {
    {
      int nameLength = static_cast<int>(cache.name.length());
      of << nameLength;
      char buf[nameLength];
      memcpy(buf, cache.name.data(), nameLength);
      for (int i = 0; i < nameLength; ++i) {
        of << buf[i];
      }
    }
    {
      int vecSize = static_cast<int>(cache.name.length());
      of << vecSize;
      for (int i = 0; i < cache.data.size(); ++i) {
        of << static_cast<float>(cache.data[i]);
      }
    }
  }
}

void Simulation::loadSavesFromDisk(const std::string &filename) {
  auto inf = std::ifstream(filename, std::ios::in | std::ios::binary);
  int size;
  inf >> size;
  simulationSaves.resize(size);

  for (auto &cache : simulationSaves) {
    {
      int nameLength;
      inf >> nameLength;
      char buf[nameLength];
      inf.read(buf, nameLength);
      cache.name = std::string(buf, buf + nameLength);
    }
    {
      int vecSize;
      inf >> vecSize;
      float buf[vecSize];
      inf.read(reinterpret_cast<char *>(buf), vecSize * sizeof(float));
      cache.data = std::vector<float>(buf, buf + vecSize);
    }
  }
}

} // namespace sapien
