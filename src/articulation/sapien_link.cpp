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

SKLink::SKLink(PxRigidDynamic *actor, SKArticulation *articulation, physx_id_t id, SScene *scene,
               std::vector<Renderer::IPxrRigidbody *> renderBodies,
               std::vector<Renderer::IPxrRigidbody *> collisionBodies)
    : SLinkBase(id, scene, renderBodies, collisionBodies), mActor(actor),
      mArticulation(articulation) {}

PxRigidDynamic *SKLink::getPxActor() { return mActor; }
SKArticulation *SKLink::getArticulation() { return mArticulation; }

} // namespace sapien
