#pragma once
#include "event_system/event_system.h"
#include "id_generator.h"
#include <PxPhysicsAPI.h>
#include <array>
#include <string>
#include <vector>

#ifdef _USE_PINOCCHIO
#include "pinocchio_model.h"
#endif

namespace sapien {
class SLinkBase;
class SJointBase;
class SScene;

enum EArticulationType { DYNAMIC, KINEMATIC };

class SArticulationBase : public EventEmitter<EventArticulationPreDestroy>,
                          public EventEmitter<EventArticulationStep> {
  std::string mName;

  int mDestroyedState{0};

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

  virtual physx::PxTransform getRootPose();
  virtual void setRootPose(const physx::PxTransform &T) = 0;

  virtual std::vector<std::array<physx::PxReal, 2>> getQlimits() const = 0;
  virtual void setQlimits(std::vector<std::array<physx::PxReal, 2>> const &v) const = 0;

  virtual void prestep() = 0;

  virtual SScene *getScene() const = 0;

  virtual ~SArticulationBase() = default;

  std::string exportKinematicsChainAsURDF(bool fixRoot);
  std::string exportURDF();

  /** internal use only, actors marked as destroyed will be removed in the next step */
  void markDestroyed();
  inline bool isBeingDestroyed() const { return mDestroyedState != 0; }
  /** internal use only, destroy has several stages, set the stage */
  inline void setDestroyedState(int state) { mDestroyedState = state; }
  /** internal use only, destroy has several stages, check which stage it is in */
  inline int getDestroyedState() const { return mDestroyedState; }

#ifdef _USE_PINOCCHIO
  std::unique_ptr<PinocchioModel> createPinocchioModel();
#endif

private:
  std::string exportTreeURDF(SLinkBase *link, physx::PxTransform extraTransform,
                             bool exportVisual = true);
};

class SArticulationDrivable : public SArticulationBase {
public:
  virtual void setDriveTarget(const std::vector<physx::PxReal> &v) = 0;
  virtual std::vector<physx::PxReal> getDriveTarget() const = 0;
};

} // namespace sapien
