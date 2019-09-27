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

  PxSimulation *mSimulation = nullptr;
  PxPhysics *mPhysicsSDK = nullptr;
  PxCooking *mCooking = nullptr;
  IPhysxRenderer *mRenderer = nullptr;
  PxClientID objectId;

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
};
