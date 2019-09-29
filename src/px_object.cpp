#include "px_object.h"

PxObject::PxObject(PxSimulation *simulation)
    : mSimulation(simulation), mPhysicsSDK(simulation->mPhysicsSDK),
      mCooking(simulation->mCooking), mRenderer(simulation->mRenderer) {
  objectId = simulation->mScene->createClient();
}

PxObject::~PxObject() {
    
}

void PxObject::addLink(PxRigidActor *newLink, const std::string &name) {
  if (!name.empty()) {
    if (namedLinks.find(name) != namedLinks.end()) {
      std::cerr << "Duplicated link name" << std::endl;
      exit(1);
    }
    newLink->setOwnerClient(objectId);
    links.push_back(newLink);
    namedLinks[name] = newLink;
  }
}

void PxObject::addJoint(PxJoint *newJoint, const std::string &name) {
  PxRigidActor *actor0;
  PxRigidActor *actor1;
  newJoint->getActors(actor0, actor1);
  if (actor0->getOwnerClient() != objectId || actor1->getOwnerClient() != objectId) {
    std::cerr << "Actor does not belong to this object" << std::endl;
    exit(1);
  }
  if (!name.empty()) {
    if (namedJoints.find(name) != namedJoints.end()) {
      std::cerr << "Duplicated link name" << std::endl;
      exit(1);
    }
    joints.push_back(newJoint);
    namedJoints[name] = newJoint;
  }
}

PxRigidActor *PxObject::getLink(PxU32 index) { return links[index]; }

PxRigidActor *PxObject::getLink(const std::string &name) { return namedLinks[name]; }

PxU32 PxObject::getNbLink() { return links.size(); }

PxJoint *PxObject::getJoint(PxU32 index) { return joints[index]; }

PxJoint *PxObject::getJoint(const std::string &name) { return namedJoints[name]; }

PxU32 PxObject::getNbJoint() { return joints.size(); }