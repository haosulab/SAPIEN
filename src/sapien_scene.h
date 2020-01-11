#pragma once
#include "id_generator.h"
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
class Simulation;
class ActorBuilder;
class ArticulationBuilder;

namespace Renderer {
class IPxrScene;
class ICamera;
} // namespace Renderer

using namespace physx;

class SScene {
  friend class ActorBuilder;
  friend class LinkBuilder;
  friend class ArticulationBuilder;

private:
  std::string mName;
  Simulation *mSimulation;             // sapien simulation
  PxScene *mPxScene;                   // physx scene
  Renderer::IPxrScene *mRendererScene; // renderer scene

  IDGenerator mLinkIdGenerator;   // assign 1 link id to each actor
  IDGenerator mRenderIdGenerator; //  assign 1 link id to each visual

  std::map<physx_id_t, std::string> mRenderId2VisualName;

  std::map<physx_id_t, SActor *> mLinkId2Actor;
  std::map<physx_id_t, SLinkBase *> mLinkId2Link;

  std::vector<std::unique_ptr<SActor>> mActors; // manages all actors
  std::vector<std::unique_ptr<SArticulation>> mArticulations;

  void addActor(std::unique_ptr<SActor> actor); // called by actor builder
  void
  addArticulation(std::unique_ptr<SArticulation> articulation); // called by articulation builder

public:
  SScene(Simulation *sim, PxScene *scene, std::string const &name = "");
  SScene(SScene const &other) = delete;
  SScene(SScene &&other) = delete;
  ~SScene();
  SScene &operator=(SScene const &other) = delete;

  inline Renderer::IPxrScene *getRendererScene() { return mRendererScene; }
  inline PxScene *getPxScene() { return mPxScene; }

  std::unique_ptr<ActorBuilder> createActorBuilder();
  std::unique_ptr<ArticulationBuilder> createArticulationBuilder();

  void removeActor(SActor *actor);
  void removeArticulation(SArticulation *articulation);

public:
  SActor *findActorById(physx_id_t id) const;
  SLinkBase *findArticulationLinkById(physx_id_t id) const;

private:
  PxReal mTimestep = 1 / 500.f;

  struct MountedCamera {
    SActorBase *actor;
    Renderer::ICamera *camera;
  };
  std::vector<MountedCamera> mCameras;

public:
  inline std::string getName() { return mName; }
  inline void setTimestep(PxReal step) { mTimestep = step; }
  inline PxReal getTimestep() { return mTimestep; }

  Renderer::ICamera *addMountedCamera(std::string const &name, SActorBase *actor,
                                      PxTransform const &pose, uint32_t width, uint32_t height,
                                      float fovx, float fovy, float near = 0.1, float far = 100);
  void removeMountedCamera(Renderer::ICamera *cam);

  void step();         // advance time by timestep
  void updateRender(); // call to sync physics world to render world

  void addGround(PxReal altitude, bool render = true, PxMaterial *material = nullptr);
};
} // namespace sapien
