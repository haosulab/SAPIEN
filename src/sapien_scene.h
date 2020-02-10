#pragma once
#include "event_system/event_system.h"
#include "id_generator.h"
#include "simulation_callback.h"
#include <PxPhysicsAPI.h>
#include <map>
#include <memory>
#include <vector>

namespace sapien {
class SActor;
class SLink;
class SLinkBase;
class SActorBase;
class SArticulation;
class SKArticulation;
class Simulation;
class ActorBuilder;
class LinkBuilder;
class ArticulationBuilder;
class SDrive;
struct SContact;

namespace Renderer {
class IPxrScene;
class ICamera;
} // namespace Renderer

namespace URDF {
class URDFLoader;
}

using namespace physx;

class SScene : public EventEmitter<EventStep> {
  friend ActorBuilder;
  friend LinkBuilder;
  friend ArticulationBuilder;

private:
  std::string mName;
  Simulation *mSimulation;             // sapien simulation
  PxScene *mPxScene;                   // physx scene
  Renderer::IPxrScene *mRendererScene; // renderer scene

  IDGenerator mLinkIdGenerator;   // assign 1 link id to each actor
  IDGenerator mRenderIdGenerator; //  assign 1 link id to each visual

  std::map<physx_id_t, std::string> mRenderId2VisualName;

  std::map<physx_id_t, SActorBase *> mLinkId2Actor;
  std::map<physx_id_t, SLinkBase *> mLinkId2Link;

  std::vector<std::unique_ptr<SActorBase>> mActors; // manages all actors
  std::vector<std::unique_ptr<SArticulation>> mArticulations;
  std::vector<std::unique_ptr<SKArticulation>> mKinematicArticulations;

  DefaultEventCallback mSimulationCallback;

  void addActor(std::unique_ptr<SActorBase> actor); // called by actor builder
  void
  addArticulation(std::unique_ptr<SArticulation> articulation); // called by articulation builder
  void addKinematicArticulation(
      std::unique_ptr<SKArticulation> articulation); // called by articulation builder

  std::vector<std::unique_ptr<SDrive>> mDrives;

public:
  SScene(Simulation *sim, PxScene *scene);
  SScene(SScene const &other) = delete;
  SScene(SScene &&other) = delete;
  ~SScene();
  SScene &operator=(SScene const &other) = delete;

  inline Simulation *getEngine() const { return mSimulation; }

  inline Renderer::IPxrScene *getRendererScene() { return mRendererScene; }
  inline PxScene *getPxScene() { return mPxScene; }

  std::unique_ptr<ActorBuilder> createActorBuilder();
  std::unique_ptr<ArticulationBuilder> createArticulationBuilder();
  std::unique_ptr<URDF::URDFLoader> createURDFLoader();

  void removeActor(SActorBase *actor);
  void removeArticulation(SArticulation *articulation);
  void removeKinematicArticulation(SKArticulation *articulation);
  void removeDrive(SDrive *drive);

  SDrive *createDrive(SActorBase *actor1, PxTransform const &pose1, SActorBase *actor2,
                      PxTransform const &pose2);

public:
  SActorBase *findActorById(physx_id_t id) const;
  SLinkBase *findArticulationLinkById(physx_id_t id) const;

private:
  PxReal mTimestep = 1 / 500.f;

  struct MountedCamera {
    SActorBase *actor;
    Renderer::ICamera *camera;
  };
  std::vector<MountedCamera> mCameras;

public:
  inline void setName(std::string const &name) { mName = name; }
  inline std::string getName() { return mName; }
  inline void setTimestep(PxReal step) { mTimestep = step; }
  inline PxReal getTimestep() { return mTimestep; }

  Renderer::ICamera *addMountedCamera(std::string const &name, SActorBase *actor,
                                      PxTransform const &pose, uint32_t width, uint32_t height,
                                      float fovx, float fovy, float near = 0.1, float far = 100);
  void removeMountedCamera(Renderer::ICamera *cam);
  Renderer::ICamera *findMountedCamera(std::string const &name, SActorBase const *actor = nullptr);
  std::vector<Renderer::ICamera *> getMountedCameras();
  std::vector<SActorBase *> getMountedActors();

  void setShadowLight(PxVec3 const &direction, PxVec3 const &color);
  void addPointLight(PxVec3 const &position, PxVec3 const &color);
  void setAmbientLight(PxVec3 const &color);
  void addDirectionalLight(PxVec3 const &direction, PxVec3 const &color);

  void step();         // advance time by TimeStep
  void updateRender(); // call to sync physics world to render world
  void addGround(PxReal altitude, bool render = true, PxMaterial *material = nullptr);

private:
  std::vector<SContact> mContacts;

  void removeMountedCameraByMount(SActorBase *actor);

public:
  void addContact(SContact const &contact);
  void clearContacts();
  std::vector<SContact> const &getContacts() const;
};
} // namespace sapien
