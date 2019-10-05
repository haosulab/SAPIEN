#pragma once
#include "id_generator.h"
#include "render_interface.h"
#include "articulation_interface.h"
#include <map>
#include <PxPhysicsAPI.h>
#include <cmath>
#include <memory>
#include <vector>

namespace sapien {
using namespace physx;

class JointSystem : public IArticulationBase {
  std::vector<PxJoint *> joints;
  std::map<std::string, PxJoint *> namedJoints;
  std::vector<PxRigidActor *> links;
  std::map<std::string, PxRigidActor *>namedLinks;
  std::vector<std::string> jointNames;
  std::vector<PxU32> jointDofs;

  class Simulation *mSimulation = nullptr;

public:
  JointSystem(Simulation *simulation);

  void addLink(PxRigidActor *newLink, const std::string &name = "", bool addToLink = true);
  
  void addJoint (PxJoint* newJoint, const std::string &name = "", bool enableCollision = false);
  
  PxRigidActor *getLink(PxU32 index);
  
  PxRigidActor *getLink(const std::string &name);
  
  PxU32 getNbLinks();

  PxJoint *getJoint(PxU32 index);

  PxJoint *getJoint(const std::string &name);

  PxU32 getNbJoints();

  // IArticulationBase
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
