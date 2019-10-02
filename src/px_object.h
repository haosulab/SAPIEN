#pragma once
#include "id_generator.h"
#include "render_interface.h"
#include "simulation.h"
#include <PxPhysicsAPI.h>
#include <cmath>
#include <memory>
#include <vector>

using namespace physx;

class PxObject {
  std::vector<PxJoint *> joints;
  std::map<std::string, PxJoint *> namedJoints;
  std::vector<PxRigidActor *> links;
  std::map<std::string, PxRigidActor *>namedLinks;
  std::vector<std::string> jointNames;
  std::vector<PxU32> jointDofs;

  PxSimulation *mSimulation = nullptr;
  PxPhysics *mPhysicsSDK = nullptr;
  PxCooking *mCooking = nullptr;
  Renderer::IPhysxRenderer *mRenderer = nullptr;

public:
  PxObject(PxSimulation *simulation);
  ~PxObject();

  void addLink(PxRigidActor *newLink, const std::string &name = "");

  void addJoint (PxJoint* newJoint, const std::string &name = "");

  PxRigidActor *getLink(PxU32 index);

  PxRigidActor *getLink(const std::string &name);

  PxU32 getNbLink();

  PxJoint *getJoint(PxU32 index);

  PxJoint *getJoint(const std::string &name);

  PxU32 getNbJoint();

  PxU32 getDofs();

  std::vector<PxU32> getJointDofs();

  std::vector<std::string> getJointNames();

  std::vector<std::tuple<physx::PxReal, physx::PxReal>> getJointLimits();

  std::vector<PxReal> getJointPositions();

  std::vector<PxReal> getJointVelocities();

  struct PxObjectWrapper *build();
};
