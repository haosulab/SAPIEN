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

class SActor : public SActorDynamicBase {
  friend ActorBuilder;

private:
  PxRigidBody *mActor = nullptr;

public:
  PxRigidBody *getPxActor() override;

  void setPose(PxTransform const &pose);

public:
  void destroy();

private:
  /* Only actor builder can create actor */
  SActor(PxRigidBody *actor, physx_id_t id, SScene *scene,
         std::vector<Renderer::IPxrRigidbody *> renderBodies);
};

class SActorStatic : public SActorBase {
  friend ActorBuilder;

private:
  PxRigidStatic *mActor = nullptr;

public:
  PxRigidActor *getPxActor() override;
  void setPose(PxTransform const &pose);

public:
  void destroy();

private:
  /* Only actor builder can create actor */
  SActorStatic(PxRigidStatic *actor, physx_id_t id, SScene *scene,
               std::vector<Renderer::IPxrRigidbody *> renderBodies);
};

} // namespace sapien
