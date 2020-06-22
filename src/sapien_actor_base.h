#pragma once
#include "sapien_shape.h"
#include "event_system/event_system.h"
#include "id_generator.h"
#include <PxPhysicsAPI.h>
#include <string>
#include <vector>

namespace sapien {
using namespace physx;

namespace Renderer {
class IPxrRigidbody;
};

class SDrive;
class SScene;

enum class EActorType {
  STATIC,
  KINEMATIC,
  DYNAMIC,
  ARTICULATION_LINK,
  KINEMATIC_ARTICULATION_LINK
};

class SActorBase : public EventEmitter<EventActorPreDestroy>, public EventEmitter<EventActorStep> {
protected:
  std::string mName = "";
  physx_id_t mId = 0;
  SScene *mParentScene = nullptr;
  std::vector<Renderer::IPxrRigidbody *> mRenderBodies;
  std::vector<Renderer::IPxrRigidbody *> mCollisionBodies;

  std::vector<SDrive *> mDrives;

  uint32_t mCol1 = 0;
  uint32_t mCol2 = 0;
  uint32_t mCol3 = 0;

  bool collisionRender = false;
  bool mHidden = false;

public:
  void renderCollisionBodies(bool collision);
  bool isRenderingCollision() const;

  void hideVisual();
  void unhideVisual();
  bool isHidingVisual() const;

  inline std::vector<SDrive *> getDrives() const { return mDrives; }

  // should not be called by users
  void addDrive(SDrive *drive);

  // should not be called by users
  void removeDrive(SDrive *drive);

  inline std::string getName() { return mName; };
  inline void setName(const std::string &name) { mName = name; }
  inline physx_id_t getId() { return mId; }
  inline SScene *getScene() { return mParentScene; }

  PxTransform getPose();
  inline uint32_t getCollisionGroup1() { return mCol1; }
  inline uint32_t getCollisionGroup2() { return mCol2; }
  inline uint32_t getCollisionGroup3() { return mCol3; }

  std::vector<SShape> getCollisionShapes();

  // render
  std::vector<Renderer::IPxrRigidbody *> getRenderBodies();
  std::vector<Renderer::IPxrRigidbody *> getCollisionBodies();
  void updateRender(PxTransform const &pose);

  virtual PxRigidActor *getPxActor() = 0;
  virtual EActorType getType() const = 0;
  virtual ~SActorBase() = default;

  // called by scene to notify a simulation step is about to happen
  virtual void prestep();

protected:
  SActorBase(physx_id_t id, SScene *scene, std::vector<Renderer::IPxrRigidbody *> renderBodies,
             std::vector<Renderer::IPxrRigidbody *> collisoinBodies);
};

class SActorDynamicBase : public SActorBase {

public:
  PxVec3 getVelocity();
  PxVec3 getAngularVelocity();
  PxReal getMass();
  PxVec3 getInertia();
  PxTransform getCMassLocalPose();

  PxRigidBody *getPxActor() override = 0;
  virtual void addForceAtPoint(PxVec3 const &force, PxVec3 const &pos);
  virtual void addForceTorque(PxVec3 const &force, PxVec3 const &torque);
  void setDamping(PxReal linear, PxReal angular);

protected:
  using SActorBase::SActorBase;
};

} // namespace sapien
