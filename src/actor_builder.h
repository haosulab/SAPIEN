#pragma once
#include "id_generator.h"
#include "render_interface.h"
#include "simulation.h"
#include <PxPhysicsAPI.h>
#include <memory>
#include <vector>

using namespace physx;

class PxActorBuilder : public std::enable_shared_from_this<PxActorBuilder> {
  PxSimulation *mSimulation = nullptr;
  PxPhysics *mPhysicsSDK = nullptr;
  PxCooking *mCooking = nullptr;
  IRenderer *mRenderer = nullptr;

  std::vector<physx_id_t> mRenderIds;
  std::vector<physx::PxShape *> mShapes;
  std::vector<physx::PxReal> mDensities;
  uint32_t mCount = 0;

public:
  PxActorBuilder(PxSimulation *simulation)
      : mSimulation(simulation), mPhysicsSDK(simulation->mPhysicsSDK),
        mCooking(simulation->mCooking), mRenderer(simulation->mRenderer) {}

  /* add convex obj */
  void
  addConvexShapeFromObj(const std::string &filename,
                        const std::string &renderFilename,
                        physx::PxTransform pose = physx::PxTransform({0,0,0}, PxIdentity),
                        PxMaterial *material = nullptr, PxReal density = 1.f);

  /* add primitive */
  void
  addPrimitiveShape(physx::PxGeometryType::Enum type,
                    physx::PxTransform pose = physx::PxTransform({0,0,0}, PxIdentity),
                    physx::PxVec3 scale = {1, 1, 1},
                    PxMaterial *material = nullptr, PxReal density = 1.f);

  PxRigidActor *build(bool isStatic = false, bool isKinematic = false);
};
