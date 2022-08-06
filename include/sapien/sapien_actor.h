#pragma once
#include "id_generator.h"
#include "sapien_actor_base.h"
#include <PxPhysicsAPI.h>
#include <string>
#include <vector>

namespace sapien {
using namespace physx;

class SDrive6D;
class SScene;
class ActorBuilder;
namespace Renderer {
class IPxrRigidbody;
}

class SActor : public SActorDynamicBase {
  friend ActorBuilder;

private:
  PxRigidDynamic *mActor = nullptr;

public:
  PxRigidDynamic *getPxActor() const override;
  void setPose(PxTransform const &pose);
  void setVelocity(PxVec3 const &v);
  void setAngularVelocity(PxVec3 const &v);
  void lockMotion(bool x, bool y, bool z, bool ax, bool ay, bool az);
  void setSolverIterations(uint32_t position, uint32_t velocity = 1);

  void setKinematicTarget(PxTransform const &pose);
  PxTransform getKinematicTarget() const;

public:
  void prestep() override;

  EActorType getType() const override;
  void destroy();

  std::vector<PxReal> packData() override;
  void unpackData(std::vector<PxReal> const &data) override;

private:
  /* Only actor builder can create actor */
  SActor(PxRigidDynamic *actor, physx_id_t id, SScene *scene,
         std::vector<Renderer::IPxrRigidbody *> renderBodies,
         std::vector<Renderer::IPxrRigidbody *> collisionBodies);
};

class SActorStatic : public SActorBase {
  friend ActorBuilder;

private:
  PxRigidStatic *mActor = nullptr;

public:
  PxRigidActor *getPxActor() const override;
  inline EActorType getType() const override { return EActorType::STATIC; }
  void setPose(PxTransform const &pose);

  std::vector<PxReal> packData() override;
  void unpackData(std::vector<PxReal> const &data) override;

public:
  void destroy();

private:
  /* Only actor builder can create actor */
  SActorStatic(PxRigidStatic *actor, physx_id_t id, SScene *scene,
               std::vector<Renderer::IPxrRigidbody *> renderBodies,
               std::vector<Renderer::IPxrRigidbody *> collisionBodies);
};

} // namespace sapien
