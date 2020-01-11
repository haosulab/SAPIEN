#pragma once
#include "id_generator.h"
#include <PxPhysicsAPI.h>
#include <string>
#include <vector>

namespace sapien {
using namespace physx;

namespace Renderer {
class IPxrRigidbody;
};

class SScene;

class SActorBase {
protected:
  std::string mName = "";
  physx_id_t mId = 0;
  SScene *mParentScene = nullptr;
  std::vector<Renderer::IPxrRigidbody *> mRenderBodies;

  uint32_t mCol1 = 0;
  uint32_t mCol2 = 0;

public:
  inline std::string getName() { return mName; };
  inline void setName(const std::string &name) { mName = name; }
  inline physx_id_t getId() { return mId; }
  inline SScene *getScene() { return mParentScene; }

  inline uint32_t getCollisionGroup1() { return mCol1; }
  inline uint32_t getCollisionGroup2() { return mCol2; }

  // render
  std::vector<Renderer::IPxrRigidbody *> getRenderBodies();
  void updateRender(PxTransform const &pose);

  virtual PxRigidActor *getPxActor() = 0;
  virtual ~SActorBase() = default;

protected:
  SActorBase(physx_id_t id, SScene *scene, std::vector<Renderer::IPxrRigidbody *> renderBodies);
};
} // namespace sapien
