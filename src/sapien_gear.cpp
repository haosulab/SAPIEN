#include "sapien/sapien_gear.h"
#include "sapien/sapien_actor.h"
#include "sapien/sapien_scene.h"
#include "sapien/simulation.h"

namespace sapien {

SGear::SGear(SScene *scene, SActorDynamicBase *actor1, PxTransform const &pose1,
             SActorDynamicBase *actor2, PxTransform const &pose2)
    : mScene(scene), mActor1(actor1), mActor2(actor2) {

  // the GearJoint is now managed by physx, sapien should not delete it.
  mGearJoint = new GearJoint(*scene->getSimulation()->mPhysicsSDK, *actor1->getPxActor(), pose1,
                             *actor2->getPxActor(), pose2);
}

float SGear::getRatio() const { return mGearJoint->getRatio(); }
void SGear::setRatio(float ratio) { mGearJoint->setRatio(ratio); }

} // namespace sapien
