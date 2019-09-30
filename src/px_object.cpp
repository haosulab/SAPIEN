#include "px_object.h"

PxObject::PxObject(PxSimulation *simulation)
    : mSimulation(simulation), mPhysicsSDK(simulation->mPhysicsSDK),
      mCooking(simulation->mCooking), mRenderer(simulation->mRenderer) {
}

PxObject::~PxObject() {
    
}

void PxObject::addLink(PxRigidActor *newLink, const std::string &name) {
  if (!name.empty()) {
    if (namedLinks.find(name) != namedLinks.end()) {
      std::cerr << "Duplicated link name" << std::endl;
      exit(1);
    }
    links.push_back(newLink);
    namedLinks[name] = newLink;
  }
}

void PxObject::addJoint(PxJoint *newJoint, const std::string &name) {
  PxRigidActor *actor0;
  PxRigidActor *actor1;
  newJoint->getActors(actor0, actor1);
  switch (newJoint->getConcreteType())
  {
    case PxJointConcreteType::eREVOLUTE: {
      jointDofs.push_back(1);
      break;
    }
    case PxJointConcreteType::ePRISMATIC: {
      jointDofs.push_back(1);
      break;
    }
    default:
      jointDofs.push_back(0);
  }
  if (!name.empty()) {
    if (namedJoints.find(name) != namedJoints.end()) {
      std::cerr << "Duplicated link name" << std::endl;
      exit(1);
    }
    joints.push_back(newJoint);
    jointNames.push_back(name);
    namedJoints[name] = newJoint;
  }
}

PxRigidActor *PxObject::getLink(PxU32 index) { return links[index]; }

PxRigidActor *PxObject::getLink(const std::string &name) { return namedLinks[name]; }

PxU32 PxObject::getNbLink() { return links.size(); }

PxJoint *PxObject::getJoint(PxU32 index) { return joints[index]; }

PxJoint *PxObject::getJoint(const std::string &name) { return namedJoints[name]; }

PxU32 PxObject::getNbJoint() { return joints.size(); }

PxU32 PxObject::getDofs() {
  PxU32 dof = 0;
  for (auto each : jointDofs) {
    dof += each;
  } 
  return dof;
}

std::vector<PxU32> PxObject::getJointDofs() {
  return jointDofs;
}

std::vector<std::string> PxObject::getJointNames() {
  return jointNames;
}

std::vector<std::tuple<physx::PxReal, physx::PxReal>> PxObject::getJointLimits() {
  std::vector<std::tuple<physx::PxReal, physx::PxReal>> limits;
  for (auto joint : joints) {
    switch (joint->getConcreteType())
    {
      case PxJointConcreteType::eREVOLUTE: {
        auto limitPair = static_cast<PxRevoluteJoint*>(joint)->getLimit();
        limits.push_back(std::make_tuple(limitPair.lower, limitPair.upper));
        break;
      }
      case PxJointConcreteType::ePRISMATIC: {
        auto limitPair = static_cast<PxRevoluteJoint*>(joint)->getLimit();
        limits.push_back(std::make_tuple(limitPair.lower, limitPair.upper));
        break;
      }
      default: {}
    }
  }
  return limits;
}


std::vector<PxReal> PxObject::getJointPositions() {
  std::vector<PxReal> positions;
  for (auto joint : joints) {
    switch (joint->getConcreteType())
    {
      case PxJointConcreteType::eREVOLUTE: {
        positions.push_back(static_cast<PxRevoluteJoint*>(joint)->getAngle());
        break;
      }
      case PxJointConcreteType::ePRISMATIC: {
        positions.push_back(static_cast<PxPrismaticJoint*>(joint)->getPosition());
        break;
      }
      default: {}
    }
  }
  return positions;
}

std::vector<PxReal> PxObject::getJointVelocities() {
  std::vector<PxReal> velocities;
  for (auto joint : joints) {
    switch (joint->getConcreteType())
    {
      case PxJointConcreteType::eREVOLUTE: {
        velocities.push_back(static_cast<PxRevoluteJoint*>(joint)->getVelocity());
        break;
      }
      case PxJointConcreteType::ePRISMATIC: {
        velocities.push_back(static_cast<PxPrismaticJoint*>(joint)->getVelocity());
        break;
      }
      default: {}
    }
  }
  return velocities;
}

void PxObject::build() {
  for (auto joint : joints) {
    joint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, false);
  }
  for (auto link : links) {
    mSimulation->mScene->addActor(*link);
  }
}
