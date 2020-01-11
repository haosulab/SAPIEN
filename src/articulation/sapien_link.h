#pragma once
#include "id_generator.h"
#include "sapien_actor_base.h"
#include <PxPhysicsAPI.h>
#include <string>
#include <vector>

namespace sapien {
using namespace physx;

class SArticulationBase;
class SArticulation;
// class SKinematicArticulation;

class SLinkBase : public SActorBase {

public:
  virtual SArticulationBase *getArticulation() = 0;

private:
  using SActorBase::SActorBase;
};

class SLink : public SLinkBase {
  friend class LinkBuilder;

private:
  PxArticulationLink *mActor = nullptr;
  SArticulation *mArticulation;

public:
  virtual SArticulationBase *getArticulation() override;
  virtual PxRigidActor *getPxActor() override;

  PxArticulationLink *getPxArticulationLink();

private:
  SLink(PxArticulationLink *actor, SArticulation *articulation, physx_id_t id, SScene *scene,
        std::vector<Renderer::IPxrRigidbody *> renderBodies);
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
