#include "kinematic_articulation.h"

KinematicArticulation::KinematicArticulation(PxSimulation *simulation)
    : mSimulation(simulation), mPhysicsSDK(simulation->mPhysicsSDK),
      mCooking(simulation->mCooking), mRenderer(simulation->mRenderer) {}

KinematicArticulation::~KinematicArticulation() {

}

KJoint *KinematicArticulation::createRevoluteJoint(KJoint *parent, PxRigidDynamic *link,
                                                   const PxTransform &poseFromParent,
                                                   const PxTransform &poseFromChild,
                                                   PxReal upperLimit, PxReal lowerLimit,
                                                   const std::string &name) {
  KJoint *newJoint = new KRevoluteJoint(link, parent, poseFromChild.getInverse(), poseFromParent,
                                        upperLimit, lowerLimit);
  joints.push_back(newJoint);
  if (!name.empty()) {
    if (namedJoints.find(name) != namedJoints.end()) {
      std::cerr << "Duplicated link name" << std::endl;
      exit(1);
    }
    namedJoints[name] = newJoint;
  }
  if (!parent) {
    root = newJoint;
  }
  return newJoint;
}

KJoint *KinematicArticulation::createContinuousJoint(KJoint *parent, PxRigidDynamic *link,
                                                     const PxTransform &poseFromParent,
                                                     const PxTransform &poseFromChild,
                                                     const std::string &name) {
  KJoint *newJoint =
      new KContinuousJoint(link, parent, poseFromChild.getInverse(), poseFromParent);
  joints.push_back(newJoint);
  if (!name.empty()) {
    if (namedJoints.find(name) != namedJoints.end()) {
      std::cerr << "Duplicated link name" << std::endl;
      exit(1);
    }
    namedJoints[name] = newJoint;
  }
  if (!parent) {
    root = newJoint;
  }
  return newJoint;
}

KJoint *KinematicArticulation::createFixedJoint(KJoint *parent, PxRigidDynamic *link,
                                                const PxTransform &poseFromParent,
                                                const PxTransform &poseFromChild,
                                                const std::string &name) {
  KJoint *newJoint = new KFixedJoint(link, parent, poseFromChild.getInverse(), poseFromParent);
  joints.push_back(newJoint);
  if (!name.empty()) {
    if (namedJoints.find(name) != namedJoints.end()) {
      std::cerr << "Duplicated link name" << std::endl;
      exit(1);
    }
    namedJoints[name] = newJoint;
  }
  if (!parent) {
    root = newJoint;
  }
  return newJoint;
}

KJoint *KinematicArticulation::createPrismaticJoint(KJoint *parent, PxRigidDynamic *link,
                                                    const PxTransform &poseFromParent,
                                                    const PxTransform &poseFromChild,
                                                    PxReal upperLimit, PxReal lowerLimit,
                                                    const std::string &name) {
  KJoint *newJoint = new KPrismaticJoint(link, parent, poseFromChild.getInverse(), poseFromParent,
                                         upperLimit, lowerLimit);
  joints.push_back(newJoint);
  if (!name.empty()) {
    if (namedJoints.find(name) != namedJoints.end()) {
      std::cerr << "Duplicated link name" << std::endl;
      exit(1);
    }
    namedJoints[name] = newJoint;
  }
  if (!parent) {
    root = newJoint;
  }
  return newJoint;
}

void KinematicArticulation::update() {
  PxReal dt = mSimulation->getTimestep();
  root->advance(dt);
  root->update();
}

void KinematicArticulation::setAllPos(const std::vector<PxReal> &v_pos) {
  PxU32 index = 0;
  for (KJoint *joint : joints) {
    if (index >= v_pos.size()) {break;}
    switch (joint->type) {
      case REVOLUTE: {
        auto rJoint = static_cast<KRevoluteJoint* >(joint);
        if (v_pos[index] < rJoint->lowerLimit || v_pos[index] > rJoint->upperLimit) {
          std::cerr << "value to set exceeds revolute joint limits" << std::endl;
        } 
        rJoint->angle = v_pos[index];
        index++;
        break;
      }
      case CONTINUOUS: {
        auto cJoint = static_cast<KContinuousJoint* >(joint);
        cJoint->angle = v_pos[index];
        index++;
        break;
      }
      case PRISMATIC: {
        auto pJoint = static_cast<KPrismaticJoint* >(joint);
        if (v_pos[index] < pJoint->lowerLimit || v_pos[index] > pJoint->upperLimit) {
          std::cerr << "value to set exceeds prismatic joint limits" << std::endl;
        } 
        pJoint->translation = v_pos[index];
        index++;
        break;
      }
    }
  }
 }

void KinematicArticulation::setAllVelocity(const std::vector<PxReal> &v_vel) {
  PxU32 index = 0;
  for (KJoint *joint : joints) {
    if (index >= v_vel.size()) {break;}
    switch (joint->type) {
      case REVOLUTE: {
        auto rJoint = static_cast<KRevoluteJoint* >(joint);
        rJoint->velocity = v_vel[index];
        index++;
        break;
      }
      case CONTINUOUS: {
        auto cJoint = static_cast<KContinuousJoint* >(joint);
        cJoint->velocity = v_vel[index];
        index++;
        break;
      }
      case PRISMATIC: {
        auto pJoint = static_cast<KPrismaticJoint* >(joint);
        pJoint->velocity = v_vel[index];
        index++;
        break;
      }
    }
  }
}

KJoint *KinematicArticulation::getJoint(PxU32 index) {
  return joints[index];
}

KJoint *KinematicArticulation::getJoint(const std::string &name) {
  return namedJoints[name];
}

PxU32 KinematicArticulation::getNbJoint() {
  return joints.size();
}