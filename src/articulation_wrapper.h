#pragma once
#include "articulation_interface.h"
#include <PxPhysicsAPI.h>
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
  std::vector<std::tuple<physx::PxReal, physx::PxReal>> jointLimits;

  // Drive specified cache
  bool balanceForce;
  std::vector<PxArticulationJointReducedCoordinate *> joints;
  std::vector<PxArticulationAxis::Enum> jointAxises;

  /* call to update cache with current articulation */
  void updateCache();
  /* call to apply cache into articulation */
  void updateArticulation();
  /* Call to update each simulation step */
  void update();

  EArticulationType get_articulation_type() const override;
  uint32_t dof() const override;

  std::vector<std::string> get_joint_names() const override;
  std::vector<uint32_t> get_joint_dofs() const override;

  std::vector<std::tuple<physx::PxReal, physx::PxReal>> get_joint_limits() const override;

  std::vector<physx::PxReal> get_qpos() const override;
  void set_qpos(const std::vector<physx::PxReal> &v) override;

  std::vector<physx::PxReal> get_qvel() const override;
  void set_qvel(const std::vector<physx::PxReal> &v) override;

  std::vector<physx::PxReal> get_qacc() const override;
  void set_qacc(const std::vector<physx::PxReal> &v) override;

  std::vector<physx::PxReal> get_qf() const override;
  void set_qf(const std::vector<physx::PxReal> &v) override;

  // Drive specific member function
  std::vector<std::string> get_drive_joint_names() const override;
  void set_drive_target(const std::vector<physx::PxReal> &v) override;
  void set_drive_property(PxReal stiffness, PxReal damping, PxReal forceLimit=PX_MAX_F32,
                          const std::vector<uint32_t> &jointIndex = {});
  void set_force_balance(bool balanceForce);

};

} // namespace sapien
