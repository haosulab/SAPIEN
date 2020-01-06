#pragma once
#include "id_generator.h"
#include <PxPhysicsAPI.h>
#include <string>
#include <vector>

namespace sapien {
using namespace physx;

class SScene;
class ActorBuilder;
namespace Renderer {
class IPxrRigidbody;
}

class SActor {
  friend ActorBuilder;

private:
  std::string mName = "";
  PxRigidActor *mActor = nullptr;
  physx_id_t mId = 0;
  SScene *mParentScene = nullptr;

  std::vector<Renderer::IPxrRigidbody *> mRenderBodies;

public:
  /* Get the name of the actor */
  inline std::string getName() { return mName; };

  /* Get the name of the actor */
  inline void setName(const std::string &name) { mName = name; }

  inline PxRigidActor *getPxActor() { return mActor; }

  std::vector<Renderer::IPxrRigidbody *> getRenderBodies();

  /* Get the id of the actor */
  inline physx_id_t getId() { return mId; }

  inline SScene *getScene() { return mParentScene; }

  void updateRender(PxTransform const &pose);

  void destroy();

private:
  /* Only scene can create actor */
  SActor(PxRigidActor *actor, physx_id_t id, SScene *scene,
         std::vector<Renderer::IPxrRigidbody *> renderBodies);
};

} // namespace sapien
