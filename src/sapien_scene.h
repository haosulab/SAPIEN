#pragma once
#include "id_generator.h"
#include <PxPhysicsAPI.h>
#include <map>
#include <memory>
#include <vector>

namespace sapien {
class SActor;
class Simulation;
class ActorBuilder;

namespace Renderer {
class IPxrScene;
class ICamera;
} // namespace Renderer

using namespace physx;

class SScene {
  friend class ActorBuilder;

private:
  Simulation *mSimulation;             // sapien simulation
  PxScene *mPxScene;                   // physx scene
  Renderer::IPxrScene *mRendererScene; // renderer scene

  IDGenerator mLinkIdGenerator;   // assign 1 link id to each actor
  IDGenerator mRenderIdGenerator; //  assign 1 link id to each visual

  std::map<physx_id_t, std::string> mRenderId2VisualName;

  std::vector<std::unique_ptr<SActor>> mActors; // manages all actors

  void addActor(std::unique_ptr<SActor> actor); // called by actor builder

public:
  SScene(Simulation *sim, PxScene *scene);
  SScene(SScene const &other) = delete;
  SScene(SScene &&other) = delete;
  ~SScene();
  SScene &operator=(SScene const &other) = delete;

  inline Renderer::IPxrScene *getRendererScene() { return mRendererScene; }
  inline PxScene *getPxScene() { return mPxScene; }

  std::unique_ptr<ActorBuilder> createActorBuilder();
  // std::unique_ptr<class ArticulationBuilder> createArticulationBuilder();
  // std::unique_ptr<class URDFLoader> createURDFLoader();
  // class ControllableArticulationWrapper *
  // createControllableArticulationWrapper(class IArticulationDrivable *baseWrapper);

  void removeActor(SActor *actor);

private:
  PxReal mTimestep = 1 / 500.f;

  struct MountedCamera {
    SActor *actor;
    Renderer::ICamera *camera;
  };
  std::vector<MountedCamera> mCameras;

public:
  inline void setTimestep(PxReal step) { mTimestep = step; }
  inline PxReal getTimestep() { return mTimestep; }

  Renderer::ICamera *addMountedCamera(std::string const &name, SActor *actor,
                                      PxTransform const &pose, uint32_t width, uint32_t height,
                                      float fovx, float fovy, float near = 0.1, float far = 100);
  void removeMountedCamera(Renderer::ICamera *cam);

  void step();         // advance time by timestep
  void updateRender(); // call to sync physics world to render world

  void addGround(PxReal altitude, bool render = true, PxMaterial *material = nullptr);
};
} // namespace sapien
