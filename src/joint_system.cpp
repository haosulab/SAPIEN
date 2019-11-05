#include "joint_system.h"
#include "common.h"
#include "simulation.h"

namespace sapien {
JointSystem::JointSystem(Simulation *simulation) : mSimulation(simulation) {}

void JointSystem::addLink(PxRigidActor *newLink, const std::string &name, bool addToScene) {
  if (!name.empty()) {
    if (namedLinks.find(name) != namedLinks.end()) {
      std::cerr << "Duplicated link name" << std::endl;
      exit(1);
    }
    links.push_back(newLink);
    namedLinks[name] = newLink;
    delete newLink->getName();
    newLink->setName(newNameFromString(name));
  }
  if (addToScene) {
    mSimulation->mScene->addActor(*newLink);
  }
}

void JointSystem::addJoint(PxJoint *newJoint, const std::string &name, bool enableCollision) {
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
    delete newJoint->getName();
    newJoint->setName(newNameFromString(name));
  }
}

PxRigidActor *JointSystem::getLink(PxU32 index) { return links[index]; }

PxRigidActor *JointSystem::getLink(const std::string &name) { return namedLinks[name]; }

PxU32 JointSystem::getNbLinks() { return links.size(); }

PxJoint *JointSystem::getJoint(PxU32 index) { return joints[index]; }

PxJoint *JointSystem::getJoint(const std::string &name) { return namedJoints[name]; }

PxU32 JointSystem::getNbJoints() { return joints.size(); }

EArticulationType JointSystem::get_articulation_type() const { return OBJECT_ARTICULATION; }

uint32_t JointSystem::dof() const {
  PxU32 dof = 0;
  for (auto each : jointDofs) {
    dof += each;
  }
  return dof;
}

std::vector<std::string> JointSystem::get_joint_names() const { return jointNames; }

std::vector<uint32_t> JointSystem::get_joint_dofs() const { return jointDofs; }

std::vector<std::array<physx::PxReal, 2>> JointSystem::get_joint_limits() const {
  std::vector<std::array<physx::PxReal, 2>> limits;
  for (auto joint : joints) {
    switch (joint->getConcreteType()) {
    case PxJointConcreteType::eREVOLUTE: {
      auto limitPair = static_cast<PxRevoluteJoint *>(joint)->getLimit();
      limits.push_back({limitPair.lower, limitPair.upper});
      break;
    }
    case PxJointConcreteType::ePRISMATIC: {
      auto limitPair = static_cast<PxRevoluteJoint *>(joint)->getLimit();
      limits.push_back({limitPair.lower, limitPair.upper});
      break;
    }
    default: {}
    }
  }
  return limits;
}

std::vector<physx::PxReal> JointSystem::get_qpos() const {
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
void JointSystem::set_qpos(const std::vector<physx::PxReal> &v) {
  std::cerr << "Setting qpos to object joint is not supported" << std::endl;
}

std::vector<physx::PxReal> JointSystem::get_qvel() const {
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

void JointSystem::set_qvel(const std::vector<physx::PxReal> &v) {
  std::cerr << "Setting qvel to Joint System is not supported" << std::endl;
}

std::vector<physx::PxReal> JointSystem::get_qacc() const {
  std::cerr << "Getting qacc of Joint System is not supported" << std::endl;
  return std::vector<physx::PxReal>(dof(), 0.f);
}

void JointSystem::set_qacc(const std::vector<physx::PxReal> &v) {
  std::cerr << "Setting qacc to Joint System is not supported" << std::endl;
}

std::vector<physx::PxReal> JointSystem::get_qf() const {
  std::cerr << "Setting qacc to Joint System is not supported" << std::endl;
  return std::vector<physx::PxReal>(dof(), 0.f);
}

void JointSystem::set_qf(const std::vector<physx::PxReal> &v) {
  std::cerr << "Setting qf to Joint System is not supported" << std::endl;
}

PxTransform JointSystem::get_link_joint_pose(uint32_t idx) const {
  std::cerr << "Getting Joint pose is not supported" << std::endl;
  return PxTransform(PxIdentity);
}

std::vector<int> JointSystem::get_link_joint_indices() const {
  std::cerr << "Getting joint indices from link is not supported" << std::endl;
  return {};
}
}
