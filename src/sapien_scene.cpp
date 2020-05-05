#include "sapien_scene.h"
#include "actor_builder.h"
#include "articulation/articulation_builder.h"
#include "articulation/sapien_articulation.h"
#include "articulation/sapien_joint.h"
#include "articulation/sapien_kinematic_articulation.h"
#include "articulation/sapien_kinematic_joint.h"
#include "articulation/sapien_link.h"
#include "articulation/urdf_loader.h"
#include "renderer/render_interface.h"
#include "sapien_actor.h"
#include "sapien_contact.h"
#include "sapien_drive.h"
#include "simulation.h"
#include <algorithm>
#include <spdlog/spdlog.h>

#ifdef _PROFILE
#include <easy/profiler.h>
#endif

namespace sapien {

SScene::SScene(Simulation *sim, PxScene *scene)
    : mSimulation(sim), mPxScene(scene), mRendererScene(nullptr), mSimulationCallback(this) {
  auto renderer = sim->getRenderer();
  if (renderer) {
    mRendererScene = renderer->createScene();
  }
  mPxScene->setSimulationEventCallback(&mSimulationCallback);
}

SScene::~SScene() {
  if (mRendererScene) {
    mSimulation->getRenderer()->removeScene(mRendererScene);
  }
  if (mPxScene) {
    mPxScene->release();
  }
}

void SScene::addActor(std::unique_ptr<SActorBase> actor) {
  mPxScene->addActor(*actor->getPxActor());
  mLinkId2Actor[actor->getId()] = actor.get();
  mActors.push_back(std::move(actor));
}

void SScene::addArticulation(std::unique_ptr<SArticulation> articulation) {
  for (auto link : articulation->getBaseLinks()) {
    mLinkId2Link[link->getId()] = link;
  }
  mPxScene->addArticulation(*articulation->getPxArticulation());
  mArticulations.push_back(std::move(articulation));
}

void SScene::addKinematicArticulation(std::unique_ptr<SKArticulation> articulation) {
  for (auto link : articulation->getBaseLinks()) {
    mLinkId2Link[link->getId()] = link;
    mPxScene->addActor(*link->getPxActor());
  }
  mKinematicArticulations.push_back(std::move(articulation));
}

void SScene::removeActor(SActorBase *actor) {
  // predestroy event
  EventActorPreDestroy e;
  e.actor = actor;
  actor->EventEmitter<EventActorPreDestroy>::emit(e);

  mLinkId2Actor.erase(actor->getId());

  // remove drives
  for (auto drive : actor->getDrives()) {
    drive->destroy();
  }

  // remove camera
  removeMountedCameraByMount(actor);

  // remove render bodies
  for (auto body : actor->getRenderBodies()) {
    mRenderId2VisualName.erase(body->getUniqueId());
    body->destroy();
  }

  // remove collision bodies
  for (auto body : actor->getCollisionBodies()) {
    body->destroy();
  }

  // remove physical bodies
  mPxScene->removeActor(*actor->getPxActor());
  actor->getPxActor()->release();

  // remove sapien actor
  mActors.erase(std::remove_if(mActors.begin(), mActors.end(),
                               [actor](auto &a) { return a.get() == actor; }),
                mActors.end());
}

void SScene::removeArticulation(SArticulation *articulation) {
  EventArticulationPreDestroy e;
  e.articulation = articulation;
  articulation->EventEmitter<EventArticulationPreDestroy>::emit(e);

  for (auto link : articulation->getBaseLinks()) {
    // predestroy event
    EventActorPreDestroy e;
    e.actor = link;
    link->EventEmitter<EventActorPreDestroy>::emit(e);

    // remove drives
    for (auto drive : link->getDrives()) {
      drive->destroy();
    }

    // remove camera
    removeMountedCameraByMount(link);

    // remove render bodies
    for (auto body : link->getRenderBodies()) {
      mRenderId2VisualName.erase(body->getUniqueId());
      body->destroy();
    }

    // remove reference
    mLinkId2Link.erase(link->getId());
  }

  // remove physical bodies
  mPxScene->removeArticulation(*articulation->getPxArticulation());
  articulation->getPxArticulation()->release();

  // remove sapien articulation
  mArticulations.erase(std::remove_if(mArticulations.begin(), mArticulations.end(),
                                      [articulation](auto &a) { return a.get() == articulation; }),
                       mArticulations.end());
}

void SScene::removeKinematicArticulation(SKArticulation *articulation) {
  EventArticulationPreDestroy e;
  e.articulation = articulation;
  articulation->EventEmitter<EventArticulationPreDestroy>::emit(e);

  for (auto link : articulation->getBaseLinks()) {
    // predestroy event
    EventActorPreDestroy e;
    e.actor = link;
    link->EventEmitter<EventActorPreDestroy>::emit(e);

    // remove drives
    for (auto drive : link->getDrives()) {
      drive->destroy();
    }

    // remove camera
    removeMountedCameraByMount(link);

    // remove render bodies
    for (auto body : link->getRenderBodies()) {
      mRenderId2VisualName.erase(body->getUniqueId());
      body->destroy();
    }

    // remove reference
    mLinkId2Link.erase(link->getId());

    // remove actor
    mPxScene->removeActor(*link->getPxActor());
    link->getPxActor()->release();
  }

  mKinematicArticulations.erase(
      std::remove_if(mKinematicArticulations.begin(), mKinematicArticulations.end(),
                     [articulation](auto &a) { return a.get() == articulation; }),
      mKinematicArticulations.end());
}

void SScene::removeDrive(SDrive *drive) {
  if (drive->mScene != this) {
    spdlog::get("SAPIEN")->error("failed to remove drive: drive is not in this scene.");
  }
  if (drive->mActor1) {
    drive->mActor1->removeDrive(drive);
  }
  if (drive->mActor2) {
    drive->mActor2->removeDrive(drive);
  }
  drive->mJoint->release();
  mDrives.erase(std::remove_if(mDrives.begin(), mDrives.end(),
                               [drive](auto &d) { return d.get() == drive; }),
                mDrives.end());
}

SActorBase *SScene::findActorById(physx_id_t id) const {
  auto it = mLinkId2Actor.find(id);
  if (it == mLinkId2Actor.end()) {
    return nullptr;
  }
  return it->second;
}

SLinkBase *SScene::findArticulationLinkById(physx_id_t id) const {
  auto it = mLinkId2Link.find(id);
  if (it == mLinkId2Link.end()) {
    return nullptr;
  }
  return it->second;
}

std::unique_ptr<ActorBuilder> SScene::createActorBuilder() {
  return std::make_unique<ActorBuilder>(this);
}

std::unique_ptr<ArticulationBuilder> SScene::createArticulationBuilder() {
  return std::make_unique<ArticulationBuilder>(this);
}

std::unique_ptr<URDF::URDFLoader> SScene::createURDFLoader() {
  return std::make_unique<URDF::URDFLoader>(this);
}

std::vector<Renderer::ICamera *> SScene::getMountedCameras() {
  std::vector<Renderer::ICamera *> cameras;
  cameras.reserve(mCameras.size());
  for (auto &mCamera : mCameras) {
    cameras.push_back(mCamera.camera);
  }
  return cameras;
}

std::vector<SActorBase *> SScene::getMountedActors() {
  std::vector<SActorBase *> actors;
  actors.reserve(mCameras.size());
  for (auto &mCamera : mCameras) {
    actors.push_back(mCamera.actor);
  }
  return actors;
}
Renderer::ICamera *SScene::addMountedCamera(std::string const &name, SActorBase *actor,
                                            PxTransform const &pose, uint32_t width,
                                            uint32_t height, float fovx, float fovy, float near,
                                            float far) {
  if (!mRendererScene) {
    spdlog::get("SAPIEN")->error("Failed to add camera: renderer is not added to simulation.");
    return nullptr;
  }
  auto cam = mRendererScene->addCamera(name, width, height, fovx, fovy, near, far);
  cam->setInitialPose(pose * PxTransform({0, 0, 0}, {-0.5, 0.5, 0.5, -0.5}));
  mCameras.push_back({actor, cam});
  return cam;
}

void SScene::removeMountedCamera(Renderer::ICamera *cam) {
  mRendererScene->removeCamera(cam);
  mCameras.erase(std::remove_if(mCameras.begin(), mCameras.end(),
                                [cam](MountedCamera &mc) { return mc.camera == cam; }),
                 mCameras.end());
}

Renderer::ICamera *SScene::findMountedCamera(std::string const &name, SActorBase const *actor) {
  auto it = std::find_if(mCameras.begin(), mCameras.end(), [name, actor](MountedCamera &cam) {
    return (actor == nullptr || cam.actor == actor) && cam.camera->getName() == name;
  });
  if (it != mCameras.end()) {
    return it->camera;
  } else {
    return nullptr;
  }
}

void SScene::setShadowLight(PxVec3 const &direction, PxVec3 const &color) {
  if (!mRendererScene) {
    spdlog::get("SAPIEN")->error("Failed to add light: renderer is not added to simulation.");
    return;
  }
  mRendererScene->setShadowLight({direction.x, direction.y, direction.z},
                                 {color.x, color.y, color.z});
}
void SScene::addPointLight(PxVec3 const &position, PxVec3 const &color) {
  if (!mRendererScene) {
    spdlog::get("SAPIEN")->error("Failed to add light: renderer is not added to simulation.");
    return;
  }
  mRendererScene->addPointLight({position.x, position.y, position.z}, {color.x, color.y, color.z});
}
void SScene::setAmbientLight(PxVec3 const &color) {
  if (!mRendererScene) {
    spdlog::get("SAPIEN")->error("Failed to add light: renderer is not added to simulation.");
    return;
  }
  mRendererScene->setAmbientLight({color.x, color.y, color.z});
}
void SScene::addDirectionalLight(PxVec3 const &direction, PxVec3 const &color) {
  if (!mRendererScene) {
    spdlog::get("SAPIEN")->error("Failed to add light: renderer is not added to simulation.");
    return;
  }
  mRendererScene->addDirectionalLight({direction.x, direction.y, direction.z},
                                      {color.x, color.y, color.z});
}

void SScene::step() {
  #ifdef _PROFILE
  EASY_BLOCK("Pre-step processing", profiler::colors::Blue);
  #endif

  clearContacts();
  for (auto &a : mActors) {
    a->prestep();
  }
  for (auto &a : mArticulations) {
    a->prestep();
  }
  for (auto &a : mKinematicArticulations) {
    a->prestep();
  }

#ifdef _PROFILE
  EASY_END_BLOCK;
  EASY_BLOCK("PhysX scene Step", profiler::colors::Red);
#endif

  mPxScene->simulate(mTimestep);
  while (!mPxScene->fetchResults(true)) {
  }

#ifdef _PROFILE
  EASY_END_BLOCK;
#endif

  EventStep event;
  event.timeStep = getTimestep();
  emit(event);
}

void SScene::stepAsync() {
  clearContacts();
  for (auto &a : mActors) {
    a->prestep();
  }
  for (auto &a : mArticulations) {
    a->prestep();
  }
  for (auto &a : mKinematicArticulations) {
    a->prestep();
  }
  mPxScene->simulate(mTimestep);
}

void SScene::stepWait() {
  while (!mPxScene->fetchResults(true)) {
  }
  EventStep event;
  event.timeStep = getTimestep();
  emit(event);
}

void SScene::updateRender() {
#ifdef _PROFILE
  EASY_FUNCTION("Update Render", profiler::colors::Magenta);
#endif
  if (!mRendererScene) {
    return;
  }
  for (auto &actor : mActors) {
    actor->updateRender(actor->getPxActor()->getGlobalPose());
  }

  for (auto &articulation : mArticulations) {
    for (auto &link : articulation->getBaseLinks()) {
      link->updateRender(link->getPxActor()->getGlobalPose());
    }
  }

  for (auto &articulation : mKinematicArticulations) {
    for (auto &link : articulation->getBaseLinks()) {
      link->updateRender(link->getPxActor()->getGlobalPose());
    }
  }

  for (auto &cam : mCameras) {
    cam.camera->setPose(cam.actor->getPxActor()->getGlobalPose());
  }
}

void SScene::addGround(PxReal altitude, bool render, PxMaterial *material,
                       Renderer::PxrMaterial const &renderMaterial) {
  createActorBuilder()->buildGround(altitude, render, material, renderMaterial, "ground");
}

void SScene::addContact(SContact const &contact) { mContacts.push_back(contact); }
void SScene::clearContacts() { mContacts.clear(); }
std::vector<SContact> const &SScene::getContacts() const { return mContacts; }

SDrive *SScene::createDrive(SActorBase *actor1, PxTransform const &pose1, SActorBase *actor2,
                            PxTransform const &pose2) {
  mDrives.push_back(std::unique_ptr<SDrive>(new SDrive(this, actor1, pose1, actor2, pose2)));
  return mDrives.back().get();
}

void SScene::removeMountedCameraByMount(SActorBase *actor) {
  for (auto &cam : mCameras) {
    if (cam.actor == actor) {
      mRendererScene->removeCamera(cam.camera);
    }
  }
  mCameras.erase(std::remove_if(mCameras.begin(), mCameras.end(),
                                [actor](MountedCamera &mc) { return mc.actor == actor; }),
                 mCameras.end());
}

std::vector<SActorBase *> SScene::getAllActors() const {
  std::vector<SActorBase *> output;
  for (auto &actor : mActors) {
    output.push_back(actor.get());
  }
  return output;
}
std::vector<SArticulationBase *> SScene::getAllArticulations() const {
  std::vector<SArticulationBase *> output;
  for (auto &articulation : mArticulations) {
    output.push_back(articulation.get());
  }
  for (auto &articulation : mKinematicArticulations) {
    output.push_back(articulation.get());
  }
  return output;
}

}; // namespace sapien
