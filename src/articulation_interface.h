#pragma once
#include "id_generator.h"
#include <PxPhysicsAPI.h>
#include <array>
#include <string>
#include <vector>

namespace sapien {
enum EArticulationType { DYNAMIC_ARTICULATION, KINEMATIC_ARTICULATION, OBJECT_ARTICULATION };

struct IArticulationBase {
  virtual EArticulationType get_articulation_type() const = 0;
  virtual uint32_t dof() const = 0;

  virtual std::vector<std::string> get_joint_names() const = 0;
  virtual std::vector<uint32_t> get_joint_dofs() const = 0;

  virtual std::vector<std::array<physx::PxReal, 2>> get_joint_limits() const = 0;

  virtual std::vector<physx::PxReal> get_qpos() const = 0;
  virtual void set_qpos(const std::vector<physx::PxReal> &v) = 0;

  virtual std::vector<physx::PxReal> get_qvel() const = 0;
  virtual void set_qvel(const std::vector<physx::PxReal> &v) = 0;

  virtual std::vector<physx::PxReal> get_qacc() const = 0;
  virtual void set_qacc(const std::vector<physx::PxReal> &v) = 0;

  virtual std::vector<physx::PxReal> get_qf() const = 0;
  virtual void set_qf(const std::vector<physx::PxReal> &v) = 0;

  virtual std::vector<physx::PxRigidBody *> get_links() const { return {}; }
  virtual std::vector<std::string> get_link_names() const { return {}; }
  virtual std::vector<physx_id_t> get_link_ids() const { return {}; }

  virtual ~IArticulationBase() = default;
  virtual physx::PxTransform get_link_joint_pose(uint32_t idx) const = 0;
};

class IArticulationDrivable : public IArticulationBase {
public:
  virtual void set_drive_target(const std::vector<physx::PxReal> &v) = 0;
  virtual std::vector<std::string> get_drive_joint_names() const = 0;
  virtual void move_base(const physx::PxTransform &T) = 0;
};

} // namespace sapien
