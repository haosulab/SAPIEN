#include "sapien_articulation_base.h"

namespace sapien {

class SKinematicArticulation : SArticulationDrivable {

public:
  virtual EArticulationType getType() const override;
  virtual uint32_t dof() const override;

  virtual std::vector<physx::PxReal> getQpos() const override;
  virtual void setQpos(std::vector<physx::PxReal> const &v) override;

  virtual std::vector<physx::PxReal> getQvel() const override;
  virtual void setQvel(std::vector<physx::PxReal> const &v) override;

  virtual std::vector<physx::PxReal> getQacc() const override;
  virtual void setQacc(std::vector<physx::PxReal> const &v) override;

  virtual std::vector<physx::PxReal> getQf() const override;
  virtual void setQf(std::vector<physx::PxReal> const &v) override;

  virtual void setDriveTarget(std::vector<physx::PxReal> const &v) override;
  virtual void moveBase(physx::PxTransform const &T) override;
};

} // namespace sapien
