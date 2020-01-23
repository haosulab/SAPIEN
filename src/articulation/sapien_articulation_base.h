#pragma once
#include "id_generator.h"
#include <PxPhysicsAPI.h>
#include <array>
#include <string>
#include <vector>

namespace sapien {
class SLinkBase;
class SJointBase;

enum EArticulationType { DYNAMIC, KINEMATIC };

class SArticulationBase {
  std::string mName;

public:
  inline void setName(std::string const &name) { mName = name; }
  inline std::string getName() { return mName; }

  virtual std::vector<SLinkBase *> getBaseLinks() = 0;
  virtual std::vector<SJointBase *> getBaseJoints() = 0;
  virtual SLinkBase *getRootLink() = 0;

  virtual EArticulationType getType() const = 0;
  virtual uint32_t dof() const = 0;

  virtual std::vector<physx::PxReal> getQpos() const = 0;
  virtual void setQpos(const std::vector<physx::PxReal> &v) = 0;

  virtual std::vector<physx::PxReal> getQvel() const = 0;
  virtual void setQvel(const std::vector<physx::PxReal> &v) = 0;

  virtual std::vector<physx::PxReal> getQacc() const = 0;
  virtual void setQacc(const std::vector<physx::PxReal> &v) = 0;

  virtual std::vector<physx::PxReal> getQf() const = 0;
  virtual void setQf(const std::vector<physx::PxReal> &v) = 0;
  virtual void setRootPose(const physx::PxTransform &T) = 0;

  virtual std::vector<std::array<physx::PxReal, 2>> getQlimits() const = 0;
  virtual void setQlimits(std::vector<std::array<physx::PxReal, 2>> const &v) const = 0;

  virtual ~SArticulationBase() = default;
};

class SArticulationDrivable : public SArticulationBase {
public:
  virtual void setDriveTarget(const std::vector<physx::PxReal> &v) = 0;
  virtual std::vector<physx::PxReal> getDriveTarget() const = 0;
};

} // namespace sapien
