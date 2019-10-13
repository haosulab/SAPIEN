#pragma once
#include "id_generator.h"
#include "render_interface.h"
#include "simulation.h"
#include <PxPhysicsAPI.h>
#include <memory>
#include <vector>

namespace sapien {
using namespace physx;

class ActorBuilder {
  Simulation *mSimulation = nullptr;
  PxPhysics *mPhysicsSDK = nullptr;
  PxCooking *mCooking = nullptr;
  Renderer::IPhysxRenderer *mRenderer = nullptr;

  std::vector<physx_id_t> mRenderIds;
  std::vector<physx::PxShape *> mShapes;
  std::vector<physx::PxReal> mDensities;
  uint32_t mCount = 0;

  physx_id_t mLinkId = 0;

public:
  ActorBuilder(ActorBuilder const &other) = delete;
  const ActorBuilder &operator=(ActorBuilder const &other) = delete;

  explicit ActorBuilder(Simulation *simulation)
      : mSimulation(simulation), mPhysicsSDK(simulation->mPhysicsSDK),
        mCooking(simulation->mCooking), mRenderer(simulation->mRenderer) {}

  /* add convex obj */
  void addConvexShapeFromObj(const std::string &filename,
                             const PxTransform &pose = {{0, 0, 0}, PxIdentity},
                             const PxVec3 &scale = {1, 1, 1}, PxMaterial *material = nullptr,
                             PxReal density = 1000.f);

  void addBoxShape(const PxTransform &pose = {{0, 0, 0}, PxIdentity},
                   const PxVec3 &size = {1, 1, 1}, PxMaterial *material = nullptr,
                   PxReal density = 1000.f);

  void addCapsuleShape(const PxTransform &pose = {{0, 0, 0}, PxIdentity}, PxReal radius = 1,
                       PxReal length = 1, PxMaterial *material = nullptr, PxReal density = 1000.f);

  void addSphereShape(const PxTransform &pose = {{0, 0, 0}, PxIdentity}, PxReal radius = 1,
                      PxMaterial *material = nullptr, PxReal density = 1000.f);

  physx_id_t addBoxVisual(const PxTransform &pose = {{0, 0, 0}, PxIdentity},
                          const PxVec3 &size = {1, 1, 1}, const PxVec3 &color = {1, 1, 1},
                          std::string const &name = "");

  physx_id_t addCapsuleVisual(const PxTransform &pose = {{0, 0, 0}, PxIdentity}, PxReal radius = 1,
                              PxReal length = 1, const PxVec3 &color = {1, 1, 1},
                              std::string const &name = "");

  physx_id_t addSphereVisual(const PxTransform &pose = {{0, 0, 0}, PxIdentity}, PxReal radius = 1,
                             const PxVec3 &color = {1, 1, 1}, std::string const &name = "");

  physx_id_t addObjVisual(const std::string &filename,
                          const PxTransform &pose = PxTransform({0, 0, 0}, PxIdentity),
                          const PxVec3 &scale = {1, 1, 1}, std::string const &name = "");

  PxRigidActor *build(bool isStatic = false, bool isKinematic = false,
                      std::string const &name = "", bool addToScene = true);
};

} // namespace sapien
