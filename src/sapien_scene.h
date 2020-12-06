#pragma once
#include "event_system/event_system.h"
#include "id_generator.h"
#include "renderer/render_interface.h"
#include "sapien_scene_config.h"
#include "simulation_callback.h"
#include <PxPhysicsAPI.h>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace sapien {
class SActor;
class SActorStatic;
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

struct SceneData {
  std::map<physx_id_t, std::vector<PxReal>> mActorData;
  std::map<physx_id_t, std::vector<PxReal>> mArticulationData;
  std::map<physx_id_t, std::vector<PxReal>> mArticulationDriveData;
};

using namespace physx;

class SScene : public EventEmitter<EventStep> {
  friend ActorBuilder;
  friend LinkBuilder;
  friend ArticulationBuilder;

private:
  // defaults
  PxMaterial *mDefaultMaterial;
  float mDefaultSleepThreshold;
  float mDefaultContactOffset;
  uint32_t mDefaultSolverIterations;
  uint32_t mDefaultSolverVelocityIterations;

public:
  inline float getDefaultSleepThreshold() const { return mDefaultSleepThreshold; }
  inline uint32_t getDefaultSolverIterations() const { return mDefaultSolverIterations; }
  inline uint32_t getDefaultSolverVelocityIterations() const {
    return mDefaultSolverVelocityIterations;
  }
  inline float getDefaultContactOffset() const { return mDefaultContactOffset; }

private:
  std::string mName;
  Simulation *mSimulation;             // sapien simulation
  PxScene *mPxScene;                   // physx scene
  Renderer::IPxrScene *mRendererScene; // renderer scene

  IDGenerator mLinkIdGenerator;   // assign 1 link id to each actor
  IDGenerator mRenderIdGenerator; //  assign 1 link id to each visual

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

private:
  bool mRequiresRemoveCleanUp1 {false};
  bool mRequiresRemoveCleanUp2 {false};

  /**
   *  call to clean up actors and articulations in being destroyed states
   *  Should be called after a step call has finished
   */
  void removeCleanUp1();
  void removeCleanUp2();

public:
  SScene(Simulation *sim, PxScene *scene, SceneConfig const &config);
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

  /** Mark an actor in a destroyed state
   *  Actors in destroyed state do not emit events
   *  The actors are truly removed at the end of this current frame
   *  (a frame starts after the last step call and ends after the future step call)
   *  use #isBeingDestroyed to see if the actor is removed in this frame
   */
  void removeActor(SActorBase *actor);

  /** Mark an articulation in a destroyed state
   *  Also marks its links
   *  Same rules as #removeActor applies
   */
  void removeArticulation(SArticulation *articulation);

  /** Mark an articulation in a destroyed state
   *  Also marks its links
   *  Same rules as #removeActor applies
   */
  void removeKinematicArticulation(SKArticulation *articulation);

  /** Remove a drive immediately */
  void removeDrive(SDrive *drive);

  SDrive *createDrive(SActorBase *actor1, PxTransform const &pose1, SActorBase *actor2,
                      PxTransform const &pose2);

  inline physx_id_t generateUniqueRenderId() { return mRenderIdGenerator.next(); };

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

  std::vector<SActorBase *> getAllActors() const;
  std::vector<SArticulationBase *> getAllArticulations() const;

  void setShadowLight(PxVec3 const &direction, PxVec3 const &color);
  void addPointLight(PxVec3 const &position, PxVec3 const &color);
  void setAmbientLight(PxVec3 const &color);
  void addDirectionalLight(PxVec3 const &direction, PxVec3 const &color);

  void step(); // advance time by TimeStep
  void stepAsync();
  void stepWait();

  void updateRender(); // call to sync physics world to render world
  SActorStatic *addGround(PxReal altitude, bool render = true, PxMaterial *material = nullptr,
                          Renderer::PxrMaterial const &renderMaterial = {});

  inline PxMaterial *getDefaultMaterial() { return mDefaultMaterial; }

  std::map<physx_id_t, std::string> findRenderId2VisualName() const;

private:
  // std::vector<SContact> mContacts;
  std::map<std::pair<PxShape *, PxShape *>, std::unique_ptr<SContact>> mContacts;

  void removeMountedCameraByMount(SActorBase *actor);

public:
  void updateContact(PxShape *shape1, PxShape *shape2, std::unique_ptr<SContact> contact);
  std::vector<SContact *> getContacts() const;

  SceneData packScene();
  void unpackScene(SceneData const &data);
};
} // namespace sapien
