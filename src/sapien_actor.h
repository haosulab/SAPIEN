#pragma once
#include "id_generator.h"
#include "sapien_actor_base.h"
#include <PxPhysicsAPI.h>
#include <string>
#include <vector>

namespace sapien {
using namespace physx;

class SDrive;
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
  void setVelocity(PxVec3 const &v);
  void setAngularVelocity(PxVec3 const &v);

public:
  EActorType getType() const override;
  void destroy();

  std::vector<PxReal> packData();
  void unpackData(std::vector<PxReal> const &data);

private:
  /* Only actor builder can create actor */
  SActor(PxRigidBody *actor, physx_id_t id, SScene *scene,
         std::vector<Renderer::IPxrRigidbody *> renderBodies,
         std::vector<Renderer::IPxrRigidbody *> collisionBodies);
};

class SActorStatic : public SActorBase {
  friend ActorBuilder;

private:
  PxRigidStatic *mActor = nullptr;

public:
  PxRigidActor *getPxActor() override;
  inline EActorType getType() const override { return EActorType::STATIC; }
  void setPose(PxTransform const &pose);

  std::vector<PxReal> packData();
  void unpackData(std::vector<PxReal> const &data);

public:
  void destroy();

private:
  /* Only actor builder can create actor */
  SActorStatic(PxRigidStatic *actor, physx_id_t id, SScene *scene,
               std::vector<Renderer::IPxrRigidbody *> renderBodies,
               std::vector<Renderer::IPxrRigidbody *> collisionBodies);
};

} // namespace sapien
