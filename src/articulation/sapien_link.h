#pragma once
#include "id_generator.h"
#include "sapien_actor_base.h"
#include "sapien_articulation.h"
#include <PxPhysicsAPI.h>
#include <string>
#include <vector>

namespace sapien {
using namespace physx;

class SArticulationBase;
class SArticulation;
// class SKinematicArticulation;

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
  PxArticulationLink *getPxActor() override;

private:
  SLink(PxArticulationLink *actor, SArticulation *articulation, physx_id_t id, SScene *scene,
        std::vector<Renderer::IPxrRigidbody *> renderBodies,
        std::vector<Renderer::IPxrRigidbody *> collisionBodies);
};

// class SKinematicLink : public SLinkBase {
// private:
//   PxRigidActor *mActor = nullptr;
//   SKinematicArticulation *mArticulation;

// public:
//   virtual SArticulationBase *getArticulation() override;
//   virtual PxRigidActor *getPxActor() override;

// private:
//   SKinematicLink(PxRigidActor *actor, SKinematicArticulation *articulation, physx_id_t id,
//                  SScene *scene, std::vector<Renderer::IPxrRigidbody *> renderBodies);
// };

} // namespace sapien
