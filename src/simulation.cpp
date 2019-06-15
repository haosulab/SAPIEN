#include "simulation.h"
#include "actor_builder.h"
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
    std::cerr << "Exiting..." << std::endl;
    exit(1);
  }

  // create scene
  PxSceneDesc sceneDesc(mPhysicsSDK->getTolerancesScale());
  sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);

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
  mFoundation->release();
}

void PxSimulation::step() {
  mScene->simulate(mTimestep);
  while (!mScene->fetchResults()) {
    // TODO: do useful stuff here
  }
}

void PxSimulation::updateRenderer() {
  for (auto actorIds : mActor2Ids) {
    PxU32 n = actorIds.first->getNbShapes();
    #ifdef _DEBUG
    if (actorIds.second.size() != n) {
      std::cerr << actorIds.second.size() << " " << n << std::endl;
      throw "Invalid shape size";
    }
    #endif
    PxShape* buffer[n];
    actorIds.first->getShapes(buffer, n);
    for (PxU32 i = 0; i < n; ++i) {
      PxTransform pose = PxShapeExt::getGlobalPose(*buffer[i], *actorIds.first);
      mRenderer->updateRigidbody(actorIds.second[i], pose);
    }
  }
}

std::unique_ptr<PxActorBuilder> PxSimulation::createActorBuilder() {
  return std::make_unique<PxActorBuilder>(this);
}

// PxActor* PxSimulation::addObj(const std::string &filename, bool isKinematic, PxVec3 p, PxQuat q) {

//   // load actor
//   PxRigidDynamic *actor =
//       mPhysicsSDK->createRigidDynamic(PxTransform(PxIdentity));
//   actor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, isKinematic);
//   PxConvexMesh* mesh = loadObjMesh(filename);
//   PxShape *shape = PxRigidActorExt::createExclusiveShape(
//       *actor, PxConvexMeshGeometry(mesh), *mDefaultMaterial);
//   if (!shape) {
//     std::cerr << "create shape failed!" << std::endl;
//     return nullptr;
//   }
//   // update density
//   PxRigidBodyExt::updateMassAndInertia(*actor, 1000);

//   actor->setGlobalPose(PxTransform(p, q));
//   mScene->addActor(*actor);

//   mScene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
//   mScene->setVisualizationParameter(PxVisualizationParameter::eACTOR_AXES, 2.0f);

//   // register actor
//   uint64_t newId = actorCounter++;
//   mActor2Id[actor] = newId;

//   // add actor for rendering
//   if (mRenderer) {
//     mRenderer->addRigidbody(newId, filename);
//   }

//   return actor;
// }


// PxActor* PxSimulation::addGroundPlane() {
//   // Create Ground
//   PxTransform pose = PxTransform(PxVec3(0.0f, 0.0f, 0.0f),
//                                  PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f)));
//   PxRigidStatic *plane = mPhysicsSDK->createRigidStatic(pose);
//   if (!plane) {
//     std::cerr << "create plane failed!" << std::endl;
//   }
//   PxShape *shape = PxRigidActorExt::createExclusiveShape(
//       *plane, PxPlaneGeometry(), *mDefaultMaterial);
//   if (!shape) {
//     std::cerr << "create shape failed!" << std::endl;
//   }
//   mScene->addActor(*plane);

//   // register actor
//   uint64_t newId = actorCounter++;
//   mActor2Id[plane] = newId;
//   if (mRenderer) {
//     mRenderer->addRigidbody(newId, PxGeometryType::ePLANE, {10, 10, 10});
//   }
//   return plane;
// }


// PxConvexMesh* PxSimulation::loadObjMesh(const std::string &filename) const {
//   std::vector<PxVec3> vertices;

//   std::ifstream f(filename);
//   std::string line;

//   std::string t;
//   float a, b, c;
//   while (std::getline(f, line)) {
//     if (line[0] == '#') {
//       continue;
//     }
//     std::istringstream iss(line);
//     iss >> t;
//     if (t == "v") {
//       iss >> a >> b >> c;
//       vertices.push_back({a, b, c});
//     }
//   }

//   PxConvexMeshDesc convexDesc;
//   convexDesc.points.count = vertices.size();
//   convexDesc.points.stride = sizeof(PxVec3);
//   convexDesc.points.data = vertices.data();
//   convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX | PxConvexFlag::eSHIFT_VERTICES;

//   PxDefaultMemoryOutputStream buf;
//   PxConvexMeshCookingResult::Enum result;
//   if (!mCooking->cookConvexMesh(convexDesc, buf, &result)) {
//     std::cerr << "Unable to cook convex mesh." << std::endl;
//     std::cerr << "Exiting..." << std::endl;
//     exit(1);
//   }
//   PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
//   PxConvexMesh *convexMesh = mPhysicsSDK->createConvexMesh(input);
//   return convexMesh;
// }
