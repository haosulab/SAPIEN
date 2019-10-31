//
// Created by sim on 10/19/19.
//
#pragma once

#include "articulation_wrapper.h"
#include "demonstration_simulation.h"
#include "optifuser_renderer.h"
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include <manipulation_trace/defination.h>

namespace sapien::robot {

enum PrimitiveCommand {
  FREE = 0,
  CartesianInterpolate = 1,
  JointInterpolate = 2,
  GripperClose = 3,
  GripperOpen = 4,
  FollowMobilityInterpolate = 5,
  CommandNumber = 6
};

enum ObjectCategory { SINGLE_DOF_OBJECT };

struct ObjectState {
  ObjectCategory category;
};

struct EndEffectorState {
  std::vector<physx::PxTransform> poses = {};
};

struct DrawerState : public ObjectState {};

struct PrimitiveAction {
  EndEffectorState eeBegin;
  EndEffectorState eeEnd;
  ObjectState objBegin;
  ObjectState objEnd;
  PrimitiveCommand command;

  bool objStateChange = false;

  PrimitiveAction(EndEffectorState eeBegin, EndEffectorState eeEnd, const ObjectState &objBegin,
                  PrimitiveCommand cmd)
      : eeBegin(std::move(eeBegin)), eeEnd(std::move(eeEnd)), objBegin(objBegin), command(cmd) {}
  void setObjectEndState(const ObjectState &state) {
    objEnd = state;
    objStateChange = true;
  }
};

struct Manipulation {
  IArticulationDrivable *articulation;
  physx::PxRigidActor *actor;
  std::vector<std::unique_ptr<PrimitiveAction>> actions;
};

class DemonstrationGUI : public Renderer::OptifuserRenderer {

public:
  DemonstrationGUI();
  void render() override;
  inline void setRobot(ArticulationWrapper *wrapper) { robot = wrapper; }
  inline void setEndEffector(const std::string &name) { eeLinkName = name; }
  inline void setSimulation(DemonstrationSimulation *sim) { simulation = sim; }

private:
  DemonstrationSimulation *simulation;
  IArticulationDrivable *robot{};
  std::string eeLinkName;
  std::vector<std::unique_ptr<Manipulation>> manipulations;

private:
  PxRigidActor *getActorTupleFromSegId(uint32_t segID);

};
} // namespace sapien::robot
