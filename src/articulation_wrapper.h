#pragma once
#include <PxPhysicsAPI.h>
#include <vector>
#include <map>
#include <string>
#include "articulation_interface.h"

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
struct PxArticulationWrapper : public IArticulationBase {
  PxArticulationReducedCoordinate *articulation = nullptr;
  PxArticulationCache *cache = nullptr;

  // cached names
  std::vector<std::string> jointNames;
  std::vector<uint32_t> jointDofs;
  std::vector<std::tuple<physx::PxReal, physx::PxReal>> jointLimits;

  /* call to update cache with current articulation */
  void updateCache();
  /* call to apply cache into articulation */
  void updateArticulation();

  virtual EArticulationType get_articulation_type() const override;
  virtual uint32_t dof() const override;

  virtual std::vector<std::string> get_joint_names() const override;
  virtual std::vector<uint32_t> get_joint_dofs() const override;

  virtual std::vector<std::tuple<physx::PxReal, physx::PxReal>> get_joint_limits() const override;

  virtual std::vector<physx::PxReal> get_qpos() const override;
  virtual void set_qpos(const std::vector<physx::PxReal> &v) override;

  virtual std::vector<physx::PxReal> get_qvel() const override;
  virtual void set_qvel(const std::vector<physx::PxReal> &v) override;

  virtual std::vector<physx::PxReal> get_qacc() const override;
  virtual void set_qacc(const std::vector<physx::PxReal> &v) override;

  virtual std::vector<physx::PxReal> get_qf() const override;
  virtual void set_qf(const std::vector<physx::PxReal> &v) override;
};

}
