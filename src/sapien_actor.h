#pragma once
#include "id_generator.h"
#include "sapien_actor_base.h"
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

class SActor : public SActorBase {
  friend ActorBuilder;

private:
  PxRigidActor *mActor = nullptr;

public:
  virtual PxRigidActor *getPxActor() override;

public:
  void destroy();

private:
  /* Only actor builder can create actor */
  SActor(PxRigidActor *actor, physx_id_t id, SScene *scene,
         std::vector<Renderer::IPxrRigidbody *> renderBodies);
};

} // namespace sapien
