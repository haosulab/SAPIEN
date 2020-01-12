#include "simulation.h"
#include "actor_builder.h"
#include "sapien_scene.h"
#include <cassert>
#include <fstream>
#include <memory>
#include <spdlog/spdlog.h>
#include <sstream>

namespace sapien {
// static PxDefaultErrorCallback gDefaultErrorCallback;
static PxDefaultAllocator gDefaultAllocatorCallback;

void SapienErrorCallback::reportError(PxErrorCode::Enum code, const char *message,
                                      const char *file, int line) {
  mLastErrorCode = code;
  spdlog::critical("{}:{}: {}", file, line, message);
}

PxErrorCode::Enum SapienErrorCallback::getLastErrorCode() {
  auto code = mLastErrorCode;
  mLastErrorCode = PxErrorCode::eNO_ERROR;
  return code;
}

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
  mDefaultMaterial = mPhysicsSDK->createMaterial(5, 5, 0.01);
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


void Simulation::setRenderer(Renderer::IPxrRenderer *renderer) {
  mRenderer = renderer;
}

PxMaterial *Simulation::createPhysicalMaterial(PxReal staticFriction, PxReal dynamicFriction,
                                               PxReal restitution) const {
  return mPhysicsSDK->createMaterial(staticFriction, dynamicFriction, restitution);
}

// class ControllableArticulationWrapper *
// Simulation::createControllableArticulationWrapper(class IArticulationDrivable *baseWrapper) {
//   auto wrapper = std::make_unique<ControllableArticulationWrapper>(baseWrapper, this);
//   wrapper->updateTimeStep(mTimestep);
//   auto wrapperPtr = wrapper.get();
//   mControllableArticulationWrapper.push_back(std::move(wrapper));
//   return wrapperPtr;
// }
// std::vector<PxReal> Simulation::dump() {
//   std::vector<PxReal> data;
//   size_t size = 0;
//   for (auto const &w : mDynamicArticulationWrappers) {
//     auto pose = w->links[0]->getGlobalPose();
//     std::vector<PxReal> poseVec = {pose.p.x, pose.p.y, pose.p.z, pose.q.w,
//                                    pose.q.x, pose.q.y, pose.q.z};
//     std::vector<PxReal> qpos(w->get_qpos());
//     std::vector<PxReal> qvel(w->get_qvel());
//     std::vector<PxReal> qacc(w->get_qacc());
//     std::vector<PxReal> qf(w->get_qf());
//     auto dof = w->dof();
//     size += 4 * dof + 7;
//     data.reserve(size);
//     data.insert(data.end(), poseVec.begin(), poseVec.end());
//     data.insert(data.end(), qpos.begin(), qpos.end());
//     data.insert(data.end(), qvel.begin(), qvel.end());
//     data.insert(data.end(), qacc.begin(), qacc.end());
//     data.insert(data.end(), qf.begin(), qf.end());
//   }
//   return data;
// }
// void Simulation::pack(const std::vector<PxReal> &data) {
//   auto begin = data.begin();
//   for (auto const &w : mDynamicArticulationWrappers) {
//     auto dof = w->dof();
//     auto poseVec = std::vector<PxReal>(begin, begin + 7);
//     PxTransform pose({poseVec[0], poseVec[1], poseVec[2]},
//                      {poseVec[4], poseVec[5], poseVec[6], poseVec[3]});
//     w->articulation->teleportRootLink(pose, true);
//     begin += 7;
//     w->set_qpos(std::vector<PxReal>(begin, begin + dof));
//     begin += dof;
//     w->set_qvel(std::vector<PxReal>(begin, begin + dof));
//     begin += dof;
//     w->set_qacc(std::vector<PxReal>(begin, begin + dof));
//     begin += dof;
//     w->set_qf(std::vector<PxReal>(begin, begin + dof));
//     begin += dof;
//     for (auto &body : w->links) {
//       body->setForceAndTorque({0, 0, 0}, {0, 0, 0});
//     }
//   }
//   assert(begin == data.end());
//   clearCache();
// }
// void Simulation::clearCache() {
//   for (auto &i : mControllableArticulationWrapper) {
//     i->clearCache();
//   }
// }

// void Simulation::writeSavesToDisk(const std::string &filename) {
//   auto of = std::ofstream(filename, std::ios::out | std::ios::binary);
//   int size = static_cast<int>(simulationSaves.size());
//   of << size;

//   for (auto &cache : simulationSaves) {
//     {
//       int nameLength = static_cast<int>(cache.name.length());
//       of << nameLength;
//       char buf[nameLength];
//       memcpy(buf, cache.name.data(), nameLength);
//       for (int i = 0; i < nameLength; ++i) {
//         of << buf[i];
//       }
//     }
//     {
//       int vecSize = static_cast<int>(cache.name.length());
//       of << vecSize;
//       for (int i = 0; i < cache.data.size(); ++i) {
//         of << static_cast<float>(cache.data[i]);
//       }
//     }
//   }
// }

// void Simulation::loadSavesFromDisk(const std::string &filename) {
//   auto inf = std::ifstream(filename, std::ios::in | std::ios::binary);
//   int size;
//   inf >> size;
//   simulationSaves.resize(size);

//   for (auto &cache : simulationSaves) {
//     {
//       int nameLength;
//       inf >> nameLength;
//       char buf[nameLength];
//       inf.read(buf, nameLength);
//       cache.name = std::string(buf, buf + nameLength);
//     }
//     {
//       int vecSize;
//       inf >> vecSize;
//       float buf[vecSize];
//       inf.read(reinterpret_cast<char *>(buf), vecSize * sizeof(float));
//       cache.data = std::vector<float>(buf, buf + vecSize);
//     }
//   }
// }

std::unique_ptr<SScene> Simulation::createScene(std::string const &name, PxVec3 gravity,
                                                PxSolverType::Enum solverType,
                                                PxSceneFlags sceneFlags) {

  PxSceneDesc sceneDesc(mPhysicsSDK->getTolerancesScale());
  sceneDesc.gravity = gravity;
  sceneDesc.filterShader = TypeAffinityFilterShader;
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

  return std::make_unique<SScene>(this, pxScene, name);
}

} // namespace sapien
