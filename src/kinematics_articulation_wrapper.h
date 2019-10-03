//
// Created by sim on 9/26/19.
//
#pragma once

#include "kinematics_joint.h"
#include <articulation_interface.h>
#include <controllable_articulation.h>
#include <iostream>
#include <map>
#include <thread>

using namespace physx;



class PxKinematicsArticulationWrapper : public IArticulationDrivable {
  KJoint *mRoot;
  std::map<std::string, std::unique_ptr<KJoint>> jointName2JointPtr = {};
  std::vector<KJoint *> jointListPtr;
  uint32_t undefinedNameGeneratorId = 0;

  bool cached = false;
  uint32_t DOF;
  uint32_t jointNum;
  std::vector<uint32_t> jointDOF;
  std::vector<std::string> jointName;
  std::vector<std::string> jointNameDOF;
  std::vector<std::size_t> jointStartIndex;
  std::vector<PxReal> qpos;
  std::vector<PxReal> qvel;
  std::vector<PxReal> qacc;
  std::vector<PxReal> qf;
  std::vector<std::tuple<PxReal, PxReal>> jointLimit;

  // Link related field
  std::vector<PxRigidDynamic *> linkListPtr;

  // Update related field
  bool updateQpos = false;
  bool updateVelocityDrive = false;
  std::vector<PxReal> driveQpos;
  std::vector<PxReal> driveQvel;

  // ROS related buffer
  ThreadSafeQueue jointStateQueue = ThreadSafeQueue();
  std::vector<ThreadSafeQueue *> positionControllerQueueList = {};
  std::vector<std::vector<uint32_t>> positionControllerIndexList = {};
  std::vector<ThreadSafeQueue *> velocityControllerQueueList = {};
  std::vector<std::vector<uint32_t>> velocityControllerIndexList = {};
  bool hasActuator = false;

public:
  EArticulationType get_articulation_type() const override {
    return EArticulationType::KINEMATIC_ARTICULATION;
  }
  void buildCache();

  KJoint *createJoint(const JointType &type, KJoint *parent, PxRigidDynamic *link,
                      const PxTransform &poseFromParent, const PxTransform &poseFromChild,
                      PxReal upperLimit = 0, PxReal lowerLimit = 0, const std::string &name = "");

  uint32_t dof() const override;
  std::vector<std::string> get_joint_names() const override;
  std::vector<uint32_t> get_joint_dofs() const override;
  std::vector<std::tuple<PxReal, PxReal>> get_joint_limits() const override;

  std::vector<physx::PxReal> get_qpos() const override;
  std::vector<physx::PxReal> get_qvel() const override;
  std::vector<physx::PxReal> get_qacc() const override;
  std::vector<physx::PxReal> get_qf() const override;

  void set_qpos(const std::vector<PxReal> &v) override;
  void set_qvel(const std::vector<PxReal> &v) override;
  void set_qacc(const std::vector<PxReal> &v) override;
  void set_qf(const std::vector<PxReal> &v) override;

  void set_drive_target(const std::vector<PxReal> &v) override;
  std::vector<std::string> get_drive_joint_name() const;

  // Customer function
  std::vector<PxRigidDynamic *> get_links();

  // This function should be called after one simulation step
  void update(PxReal timestep);

  // ROS related function
  ThreadSafeQueue *get_queue();
  void add_position_controller(const std::vector<std::string> &name, ThreadSafeQueue *queue);
  void add_velocity_controller(const std::vector<std::string> &name, ThreadSafeQueue *queue);
};
