/**
 * Sapien class for PxScene.
 *
 * Notes:
 * 1. SScene maintains a shared pointer to Simulation, in case Simulation is deleted before itself.
 * 2. Removing an actor or articulation is tricky.
 * Since PhysX contact info can contain objects removed in the current step,
 * we need to enable a cache to avoid losing the mapping between PhysX objects and Sapien objects.
 *
 * References:
 * https://documentation.help/NVIDIA-PhysX-SDK-Guide/ScenesAndActors.html#scenesandactors (PhysX 3)
 */

#pragma once

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <PxPhysicsAPI.h>

#include "event_system/event_system.h"
#include "id_generator.h"
#include "renderer/render_interface.h"
#include "sapien_light.h"
#include "sapien_material.h"
#include "sapien_scene_config.h"
#include "simulation_callback.h"

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
class SDrive6D;
class SDrive;
struct SContact;

namespace Renderer {
class IPxrScene;
class ICamera;
} // namespace Renderer

namespace URDF {
class URDFLoader;
} // namespace URDF

using namespace physx;

struct SceneData {
  std::map<physx_id_t, std::vector<PxReal>> mActorData;
  std::map<physx_id_t, std::vector<PxReal>> mArticulationData;
  std::map<physx_id_t, std::vector<PxReal>> mArticulationDriveData;
};

class SScene : public EventEmitter<EventSceneStep> {
  friend ActorBuilder;
  friend LinkBuilder;
  friend ArticulationBuilder;

  /************************************************
   * Basic
   ***********************************************/
public:
  SScene(std::shared_ptr<Simulation> sim, PxScene *scene, SceneConfig const &config);
  SScene(SScene const &other) = delete;
  SScene(SScene &&other) = delete;
  ~SScene();
  SScene &operator=(SScene const &other) = delete;

  inline std::shared_ptr<Simulation> getSimulation() const { return mSimulationShared; }
  inline PxScene *getPxScene() { return mPxScene; }

  inline Renderer::IPxrScene *getRendererScene() { return mRendererScene; }

  // default parameters of simulation solver
  inline float getDefaultSleepThreshold() const { return mDefaultSleepThreshold; }
  inline uint32_t getDefaultSolverIterations() const { return mDefaultSolverIterations; }
  inline uint32_t getDefaultSolverVelocityIterations() const {
    return mDefaultSolverVelocityIterations;
  }
  inline float getDefaultContactOffset() const { return mDefaultContactOffset; }

  // default physical material
  inline std::shared_ptr<SPhysicalMaterial> getDefaultMaterial() const { return mDefaultMaterial; }
  inline void setDefaultMaterial(std::shared_ptr<SPhysicalMaterial> material) {
    mDefaultMaterial = material;
  }

private:
  std::shared_ptr<Simulation> mSimulationShared; // shared pointer to sapien simulation
  PxScene *mPxScene;                             // physx scene
  DefaultEventCallback mSimulationCallback;      // physx scene's simulation callback

  Renderer::IPxrScene *mRendererScene; // renderer scene

  // defaults
  float mDefaultSleepThreshold;
  float mDefaultContactOffset;
  uint32_t mDefaultSolverIterations;
  uint32_t mDefaultSolverVelocityIterations;
  std::shared_ptr<SPhysicalMaterial> mDefaultMaterial;

public:
  inline void setName(std::string const &name) { mName = name; }
  inline std::string getName() { return mName; }
  inline void setTimestep(PxReal step) { mTimestep = step; }
  inline PxReal getTimestep() { return mTimestep; }

  void step(); // advance time by TimeStep
  void stepAsync();
  void stepWait();

private:
  PxReal mTimestep = 1 / 500.f;
  std::string mName;

  /************************************************
   * Physical Objects
   ***********************************************/
public:
  std::shared_ptr<SPhysicalMaterial>
  createPhysicalMaterial(PxReal staticFriction, PxReal dynamicFriction, PxReal restitution) const;

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

  SDrive6D *createDrive(SActorBase *actor1, PxTransform const &pose1, SActorBase *actor2,
                        PxTransform const &pose2);
  /** Remove a drive immediately */
  void removeDrive(SDrive *drive);

  SActorBase *findActorById(physx_id_t id) const;
  SLinkBase *findArticulationLinkById(physx_id_t id) const;
  inline physx_id_t generateUniqueRenderId() { return mRenderIdGenerator.next(); };

private:
  void addActor(std::unique_ptr<SActorBase> actor); // called by actor builder
  void
  addArticulation(std::unique_ptr<SArticulation> articulation); // called by articulation builder
  void addKinematicArticulation(
      std::unique_ptr<SKArticulation> articulation); // called by articulation builder

  bool mRequiresRemoveCleanUp1{false};
  bool mRequiresRemoveCleanUp2{false};

  /**
   *  call to clean up actors and articulations in being destroyed states
   *  Should be called after a step call has finished
   */
  void removeCleanUp1();
  void removeCleanUp2();

  IDGenerator mActorIdGenerator;  // unique id generator for actors (including links)
  IDGenerator mRenderIdGenerator; //  unique id generator for visuals

  std::map<physx_id_t, SActorBase *> mActorId2Actor;
  std::map<physx_id_t, SLinkBase *> mActorId2Link;

  std::vector<std::unique_ptr<SActorBase>> mActors; // manages all actors
  std::vector<std::unique_ptr<SArticulation>> mArticulations;
  std::vector<std::unique_ptr<SKArticulation>> mKinematicArticulations;

  std::vector<std::unique_ptr<SLight>> mLights;

  std::vector<std::unique_ptr<SDrive>> mDrives;

  /************************************************
   * Sensor
   ***********************************************/
private:
public:
  Renderer::ICamera *addMountedCamera(std::string const &name, SActorBase *actor,
                                      PxTransform const &pose, uint32_t width, uint32_t height,
                                      float fovx, float fovy, float near = 0.1, float far = 100);
  void removeMountedCamera(Renderer::ICamera *cam);
  Renderer::ICamera *findMountedCamera(std::string const &name, SActorBase const *actor = nullptr);
  std::vector<Renderer::ICamera *> getMountedCameras();
  std::vector<SActorBase *> getMountedActors();

  std::vector<SActorBase *> getAllActors() const;
  std::vector<SArticulationBase *> getAllArticulations() const;
  std::vector<SLight *> getAllLights() const;

  void setAmbientLight(PxVec3 const &color);
  PxVec3 getAmbientLight() const;
  SPointLight *addPointLight(PxVec3 const &position, PxVec3 const &color, bool enableShadow,
                             float shadowNear, float shadowFar);
  SDirectionalLight *addDirectionalLight(PxVec3 const &direction, PxVec3 const &color,
                                         bool enableShadow, PxVec3 const &position,
                                         float shadowScale, float shadowNear, float shadowFar);
  // SSpotLight *addSpotLight(PxVec3 const &position, PxVec3 const &direction, float fov,
  //                          PxVec3 const &color, bool enableShadow, float shadowNear,
  //                          float shadowFar);
  SSpotLight *addSpotLight(PxVec3 const &position, PxVec3 const &direction, float fovInner,
                           float fovOuter, PxVec3 const &color, bool enableShadow,
                           float shadowNear, float shadowFar);
  SActiveLight *addActiveLight(PxTransform const &pose, PxVec3 const &color, float fov,
                               std::string_view texPath);

  void removeLight(SLight *light);

  /** syncs physical scene with renderer scene, and tell the renderer scene that
   * it is a new time frame.
   *
   * when optical flow or motion blur is desired, you need to call this function
   * every frame even if you do not render the frame to update the model
   * matrices of objects.
   */
  void updateRender();
  SActorStatic *addGround(PxReal altitude, bool render = true,
                          std::shared_ptr<SPhysicalMaterial> material = nullptr,
                          std::shared_ptr<Renderer::IPxrMaterial> renderMaterial = nullptr);

  std::map<physx_id_t, std::string> findRenderId2VisualName() const;

private:
  void removeMountedCameraByMount(SActorBase *actor);

  struct MountedCamera {
    SActorBase *actor;
    Renderer::ICamera *camera;
  };
  std::vector<MountedCamera> mCameras;

  /************************************************
   * Contact
   ***********************************************/
public:
  void updateContact(std::unique_ptr<SContact> contact);
  std::vector<SContact *> getContacts() const;

  SceneData packScene();
  void unpackScene(SceneData const &data);

private:
  std::map<std::pair<PxShape *, PxShape *>, std::unique_ptr<SContact>> mContacts;
};
} // namespace sapien
