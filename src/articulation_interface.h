#pragma once
#include <PxPhysicsAPI.h>
#include <string>
#include <vector>

enum EArticulationType { DYNAMIC_ARTICULATION, KINEMATIC_ARTICULATION, OBJECT_ARTICULATION };

struct IArticulationBase {
  virtual EArticulationType get_articulation_type() const = 0;
  virtual uint32_t dof() const = 0;

  virtual std::vector<std::string> get_joint_names() const = 0;
  virtual std::vector<uint32_t> get_joint_dofs() const = 0;

  virtual std::vector<std::tuple<physx::PxReal, physx::PxReal>> get_joint_limits() const = 0;

  virtual std::vector<physx::PxReal> get_qpos() const = 0;
  virtual void set_qpos(const std::vector<physx::PxReal> &v) = 0;

  virtual std::vector<physx::PxReal> get_qvel() const = 0;
  virtual void set_qvel(const std::vector<physx::PxReal> &v) = 0;

  virtual std::vector<physx::PxReal> get_qacc() const = 0;
  virtual void set_qacc(const std::vector<physx::PxReal> &v) = 0;

  virtual std::vector<physx::PxReal> get_qf() const = 0;
  virtual void set_qf(const std::vector<physx::PxReal> &v) = 0;
};

class IArticulationDrivable : public IArticulationBase {
public:
  virtual void set_drive_target(const std::vector<physx::PxReal> &v) = 0;
  virtual std::vector<std::string> get_drive_joint_name() const = 0;
};
