#pragma once
#include "sapien/id_generator.h"
#include "sapien/sapien_actor_base.h"
#include "sapien_articulation.h"
#include "sapien_kinematic_articulation.h"
#include <PxPhysicsAPI.h>
#include <string>
#include <vector>

namespace sapien {
using namespace physx;

class SArticulationBase;

class SLinkBase : public SActorDynamicBase {
protected:
  uint32_t mIndex; // set when "build" from articulation builder

public:
  virtual SArticulationBase *getArticulation() = 0;
  inline uint32_t getIndex() { return mIndex; };

private:
  using SActorDynamicBase::SActorDynamicBase;
};

class SLink : public SLinkBase {
  friend class LinkBuilder;

private:
  PxArticulationLink *mActor = nullptr;
  SArticulation *mArticulation;

public:
  SArticulation *getArticulation() override;
  inline EActorType getType() const override { return EActorType::ARTICULATION_LINK; }
  PxArticulationLink *getPxActor() const override;

private:
  SLink(PxArticulationLink *actor, SArticulation *articulation, physx_id_t id, SScene *scene,
        std::vector<Renderer::IPxrRigidbody *> renderBodies,
        std::vector<Renderer::IPxrRigidbody *> collisionBodies);
};

class SKLink : public SLinkBase {
  friend class LinkBuilder;

private:
  PxRigidDynamic *mActor = nullptr;
  SKArticulation *mArticulation;

public:
  virtual SKArticulation *getArticulation() override;
  inline EActorType getType() const override { return EActorType::KINEMATIC_ARTICULATION_LINK; }
  virtual PxRigidDynamic *getPxActor() const override;

  SKLink(SKLink const &) = delete;
  SKLink &operator=(SKLink const &) = delete;
  ~SKLink() = default;

private:
  SKLink(PxRigidDynamic *actor, SKArticulation *articulation, physx_id_t id, SScene *scene,
         std::vector<Renderer::IPxrRigidbody *> renderBodies,
         std::vector<Renderer::IPxrRigidbody *> collisionBodies);
};

} // namespace sapien
