#pragma once
#include "articulation_interface.h"
#include "id_generator.h"
#include <PxPhysicsAPI.h>
#include <array>
#include <map>
#include <string>
#include <vector>

namespace sapien {
#define APP_ASSERT_FATAL(exp, msg)                                                                \
  if (!(exp)) {                                                                                   \
    std::cerr << msg << std::endl;                                                                \
    exit(1);                                                                                      \
  }

#define APP_ASSERT_WAN(exp, msg)                                                                  \
  if (!(exp)) {                                                                                   \
    std::cerr << msg << std::endl;                                                                \
  }

using namespace physx;

// TODO: proof read and test this struct
struct ArticulationWrapper : public IArticulationDrivable {
  PxArticulationReducedCoordinate *articulation = nullptr;
  PxArticulationCache *cache = nullptr;

  // cached names
  std::vector<std::string> jointNames;
  std::vector<std::string> jointNamesDOF;
  std::vector<uint32_t> jointDofs;
  std::vector<std::array<physx::PxReal, 2>> jointLimits;

  // Drive cache
  bool balanceForce;
  std::vector<PxArticulationJointReducedCoordinate *> activeJoints;
  std::vector<PxArticulationAxis::Enum> jointAxises;
  std::vector<std::string> jointTypes;

  // Actuator cache
  // TODO: support multi-dof joint
  std::vector<std::string> forceActuatorName;
  std::vector<uint32_t> forceActuatorIndex;
  std::vector<std::array<PxReal, 2>> forceActuatorLimit;

  // Link cache
  std::map<std::string, PxArticulationLink *> linkName2Link;
  std::vector<PxArticulationLink *> links;
  std::vector<PxReal> linkMasses;
  std::vector<PxVec3> linkInertial;
  std::vector<physx_id_t> linkSegmentationIds;
  std::vector<int> link2JointIndices;

  /* call to update cache with current articulation */
  void updateCache();
  /* call to apply cache into articulation */
  void updateArticulation();
  /* Call to update each simulation step */
  void update();

  // TODO: Planed interface
  virtual std::vector<physx::PxRigidBody *> get_links() const override;
  virtual std::vector<std::string> get_link_names() const override;
  virtual std::vector<physx_id_t> get_link_ids() const override;
  virtual std::vector<int> get_link_joint_indices() const override;

  // IArticulationBase
  virtual EArticulationType get_articulation_type() const override;
  virtual uint32_t dof() const override;

  virtual std::vector<std::string> get_joint_names() const override;
  virtual std::vector<uint32_t> get_joint_dofs() const override;
  virtual std::vector<std::string> get_joint_types() const override;

  virtual std::vector<std::array<physx::PxReal, 2>> get_qlimits() const override;

  virtual std::vector<physx::PxReal> get_qpos() const override;
  virtual void set_qpos(const std::vector<physx::PxReal> &v) override;

  virtual std::vector<physx::PxReal> get_qvel() const override;
  virtual void set_qvel(const std::vector<physx::PxReal> &v) override;

  virtual std::vector<physx::PxReal> get_qacc() const override;
  virtual void set_qacc(const std::vector<physx::PxReal> &v) override;

  virtual std::vector<physx::PxReal> get_qf() const override;
  virtual void set_qf(const std::vector<physx::PxReal> &v) override;

  // Drive specific member function
  virtual std::vector<std::string> get_qnames() const override;
  virtual void set_drive_target(const std::vector<physx::PxReal> &v) override;
  void set_drive_velocity_target(const std::vector<physx::PxReal> &v,
                                 const std::vector<uint32_t> &index);

  virtual PxTransform get_link_joint_pose(uint32_t idx) const override;

  void set_drive_property(PxReal stiffness, PxReal damping, PxReal forceLimit = PX_MAX_F32,
                          const std::vector<uint32_t> &jointIndex = {});
  void set_force_balance(bool balanceForce);
  void move_base(const PxTransform &newT) override;

  // Mimic the Mujoco actuator modeling functions
  void addForceActuator(const std::string &jointName, PxReal lowerLimit, PxReal upperLimit);
  std::vector<std::array<PxReal, 2>> const &getForceActuatorRanges() const;
  std::vector<std::string> const &getForceActuatorNames() const;
  void applyActuatorForce(const std::vector<physx::PxReal> &v);

  // Link based function
  std::vector<std::array<PxReal, 6>> get_cfrc_ext();
};

} // namespace sapien
