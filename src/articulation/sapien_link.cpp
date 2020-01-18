#include "sapien_link.h"
#include "sapien_articulation.h"

namespace sapien {

PxArticulationLink *SLink::getPxActor() { return mActor; }
SArticulation *SLink::getArticulation() { return mArticulation; }

SLink::SLink(PxArticulationLink *actor, SArticulation *articulation, physx_id_t id, SScene *scene,
             std::vector<Renderer::IPxrRigidbody *> renderBodies,
             std::vector<Renderer::IPxrRigidbody *> collisionBodies)
    : SLinkBase(id, scene, renderBodies, collisionBodies), mActor(actor),
      mArticulation(articulation) {}

// PxRigidActor *SKinematicLink::getPxActor() { return mActor; }
// SArticulationBase *SKinematicLink::getArticulation() { return mArticulation; }
// SKinematicLink::SKinematicLink(PxRigidActor *actor, SKinematicArticulation *articulation,
//                                physx_id_t id, SScene *scene,
//                                std::vector<Renderer::IPxrRigidbody *> renderBodies)
//     : SLinkBase(id, scene, renderBodies), mActor(actor), mArticulation(articulation) {}

} // namespace sapien
