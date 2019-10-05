#include "joint_system.h"
#include "simulation.h"

namespace sapien {
PxJointSystem::PxJointSystem(Simulation *simulation) : mSimulation(simulation) {}

void PxJointSystem::addLink(PxRigidActor *newLink, const std::string &name, bool addToScene) {
  if (!name.empty()) {
    if (namedLinks.find(name) != namedLinks.end()) {
      std::cerr << "Duplicated link name" << std::endl;
      exit(1);
    }
    links.push_back(newLink);
    namedLinks[name] = newLink;
    newLink->setName(name.c_str());
  }
  if (addToScene) {
    mSimulation->mScene->addActor(*newLink);
  }
}

void PxJointSystem::addJoint(PxJoint *newJoint, const std::string &name, bool enableCollision) {
  PxRigidActor *actor0;
  PxRigidActor *actor1;
  newJoint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, enableCollision);
  newJoint->getActors(actor0, actor1);
  switch (newJoint->getConcreteType()) {
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
    newJoint->setName(name.c_str());
  }
}

PxRigidActor *PxJointSystem::getLink(PxU32 index) { return links[index]; }

PxRigidActor *PxJointSystem::getLink(const std::string &name) { return namedLinks[name]; }

PxU32 PxJointSystem::getNbLinks() { return links.size(); }

PxJoint *PxJointSystem::getJoint(PxU32 index) { return joints[index]; }

PxJoint *PxJointSystem::getJoint(const std::string &name) { return namedJoints[name]; }

PxU32 PxJointSystem::getNbJoints() { return joints.size(); }

EArticulationType PxJointSystem::get_articulation_type() const { return OBJECT_ARTICULATION; }

uint32_t PxJointSystem::dof() const {
  PxU32 dof = 0;
  for (auto each : jointDofs) {
    dof += each;
  }
  return dof;
}

std::vector<std::string> PxJointSystem::get_joint_names() const { return jointNames; }

std::vector<uint32_t> PxJointSystem::get_joint_dofs() const { return jointDofs; }

std::vector<std::tuple<physx::PxReal, physx::PxReal>> PxJointSystem::get_joint_limits() const {
  std::vector<std::tuple<physx::PxReal, physx::PxReal>> limits;
  for (auto joint : joints) {
    switch (joint->getConcreteType()) {
    case PxJointConcreteType::eREVOLUTE: {
      auto limitPair = static_cast<PxRevoluteJoint *>(joint)->getLimit();
      limits.push_back(std::make_tuple(limitPair.lower, limitPair.upper));
      break;
    }
    case PxJointConcreteType::ePRISMATIC: {
      auto limitPair = static_cast<PxRevoluteJoint *>(joint)->getLimit();
      limits.push_back(std::make_tuple(limitPair.lower, limitPair.upper));
      break;
    }
    default: {}
    }
  }
  return limits;
}

std::vector<physx::PxReal> PxJointSystem::get_qpos() const {
  std::vector<PxReal> positions;
  for (auto joint : joints) {
    switch (joint->getConcreteType()) {
    case PxJointConcreteType::eREVOLUTE: {
      positions.push_back(static_cast<PxRevoluteJoint *>(joint)->getAngle());
      break;
    }
    case PxJointConcreteType::ePRISMATIC: {
      positions.push_back(static_cast<PxPrismaticJoint *>(joint)->getPosition());
      break;
    }
    default: {}
    }
  }
  return positions;
}
void PxJointSystem::set_qpos(const std::vector<physx::PxReal> &v) {
  std::cerr << "Setting qpos to object joint is not supported" << std::endl;
}

std::vector<physx::PxReal> PxJointSystem::get_qvel() const {
  std::vector<PxReal> velocities;
  for (auto joint : joints) {
    switch (joint->getConcreteType()) {
    case PxJointConcreteType::eREVOLUTE: {
      velocities.push_back(static_cast<PxRevoluteJoint *>(joint)->getVelocity());
      break;
    }
    case PxJointConcreteType::ePRISMATIC: {
      velocities.push_back(static_cast<PxPrismaticJoint *>(joint)->getVelocity());
      break;
    }
    default: {}
    }
  }
  return velocities;
}

void PxJointSystem::set_qvel(const std::vector<physx::PxReal> &v) {
  std::cerr << "Setting qvel to Joint System is not supported" << std::endl;
}

std::vector<physx::PxReal> PxJointSystem::get_qacc() const {
  std::cerr << "Getting qacc of Joint System is not supported" << std::endl;
  return std::vector<physx::PxReal>(dof(), 0.f);
}

void PxJointSystem::set_qacc(const std::vector<physx::PxReal> &v) {
  std::cerr << "Setting qacc to Joint System is not supported" << std::endl;
}

std::vector<physx::PxReal> PxJointSystem::get_qf() const {
  std::cerr << "Setting qacc to Joint System is not supported" << std::endl;
  return std::vector<physx::PxReal>(dof(), 0.f);
}

void PxJointSystem::set_qf(const std::vector<physx::PxReal> &v) {
  std::cerr << "Setting qf to Joint System is not supported" << std::endl;
}

}
