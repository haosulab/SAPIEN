//
// Created by sim on 9/26/19.
//
#pragma once

#include "kinematics_joint.h"
#include <articulation_interface.h>
#include <controllable_articulation_wrapper.h>
#include <iostream>
#include <map>
#include <thread>

namespace sapien {
class Simulation;
using namespace physx;

class KinematicsArticulationWrapper : public IArticulationDrivable {
  KJoint *mRoot;
  std::map<std::string, std::shared_ptr<KJoint>> jointName2JointPtr = {};
  std::vector<KJoint *> jointListPtr;
  uint32_t undefinedNameGeneratorId = 0;

  // Cache
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
  std::vector<std::array<PxReal, 2>> jointLimit;

  // Link related field
  std::vector<PxRigidDynamic *> linkListPtr;
  std::vector<std::string> linkNames;
  std::vector<physx_id_t> linkIds;

  // Update related field
  PxTransform rootPose = PxTransform(PxIdentity);
  bool hasMagicVelocity = false;
  std::vector<PxReal> lastStepQpos;
  std::vector<PxReal> driveQvel;

  Simulation *mSimulation;

public:
  KinematicsArticulationWrapper(Simulation &sim);
  EArticulationType get_articulation_type() const override;
  void buildCache();

  KJoint *createJoint(const JointType &type, KJoint *parent, PxRigidDynamic *link,
                      const PxTransform &poseFromParent, const PxTransform &poseFromChild,
                      PxReal upperLimit = 0, PxReal lowerLimit = 0, const std::string &name = "");

  uint32_t dof() const override;
  std::vector<std::string> get_joint_names() const override;
  std::vector<uint32_t> get_joint_dofs() const override;
  std::vector<std::array<PxReal, 2>> get_joint_limits() const override;

  std::vector<physx::PxReal> get_qpos() const override;
  std::vector<physx::PxReal> get_qvel() const override;
  std::vector<physx::PxReal> get_qacc() const override;
  std::vector<physx::PxReal> get_qf() const override;

  void set_qpos(const std::vector<PxReal> &v) override;
  void set_qvel(const std::vector<PxReal> &v) override;
  void set_qacc(const std::vector<PxReal> &v) override;
  void set_qf(const std::vector<PxReal> &v) override;

  void set_drive_target(const std::vector<PxReal> &v) override;
  std::vector<std::string> get_drive_joint_names() const override;

  physx::PxTransform get_link_joint_pose(uint32_t idx) const override;

  // Customer function
  inline std::vector<PxRigidBody *> get_links() const override {
    return std::vector<PxRigidBody *>(linkListPtr.begin(), linkListPtr.end());
  };

  std::vector<std::string> get_link_names() const override;
  std::vector<physx_id_t> get_link_ids() const override;
  std::vector<int> get_link_joint_indices() const override;

  // This function should be called after one simulation step
  void update(PxReal timestep);
  void move_base(const PxTransform &T) override;
};

} // namespace sapien
