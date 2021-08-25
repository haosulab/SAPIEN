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

#include <easy/profiler.h>

namespace sapien {

/************************************************
 * Basic
 ***********************************************/
SScene::SScene(std::shared_ptr<Simulation> sim, PxScene *scene, SceneConfig const &config)
    : mSimulationShared(sim), mPxScene(scene), mSimulationCallback(this), mRendererScene(nullptr) {

  // default parameters for physical materials, contact solver, etc.
  mDefaultMaterial =
      createPhysicalMaterial(config.static_friction, config.dynamic_friction, config.restitution);
  mDefaultContactOffset = config.contactOffset;
  mDefaultSleepThreshold = config.sleepThreshold;
  mDefaultSolverIterations = config.solverIterations;
  mDefaultSolverVelocityIterations = config.solverVelocityIterations;

  mPxScene->setSimulationEventCallback(&mSimulationCallback);

  auto renderer = sim->getRenderer();
  if (renderer) {
    mRendererScene = renderer->createScene();
  }
}

SScene::~SScene() {
  mDefaultMaterial.reset();
  for (auto &actor : mActors) {
    actor->getPxActor()->release();
  }
  for (auto &articulation : mArticulations) {
    articulation->getPxArticulation()->release();
  }
  for (auto &ka : mKinematicArticulations) {
    for (auto &link : ka->getBaseLinks()) {
      link->getPxActor()->release();
    }
  }
  for (auto &drive : mDrives) {
    drive.release();
  }
  mPxScene->release();

  // TODO: check whether we implement mXXX.release() to replace the workaround
  mActors.clear();
  mArticulations.clear();
  mKinematicArticulations.clear();

  if (mRendererScene) {
    mSimulationShared->getRenderer()->removeScene(mRendererScene);
  }

  // Finally, release the shared pointer to simulation
  mSimulationShared.reset();
}

/************************************************
 * Create objects
 ***********************************************/
std::shared_ptr<SPhysicalMaterial> SScene::createPhysicalMaterial(PxReal staticFriction,
                                                                  PxReal dynamicFriction,
                                                                  PxReal restitution) const {
  return mSimulationShared->createPhysicalMaterial(staticFriction, dynamicFriction, restitution);
}

std::shared_ptr<ActorBuilder> SScene::createActorBuilder() {
  return std::make_shared<ActorBuilder>(this);
}

std::shared_ptr<ArticulationBuilder> SScene::createArticulationBuilder() {
  return std::make_shared<ArticulationBuilder>(this);
}

std::unique_ptr<URDF::URDFLoader> SScene::createURDFLoader() {
  return std::make_unique<URDF::URDFLoader>(this);
}

SDrive6D *SScene::createDrive(SActorBase *actor1, PxTransform const &pose1, SActorBase *actor2,
                              PxTransform const &pose2) {
  mDrives.push_back(std::unique_ptr<SDrive6D>(new SDrive6D(this, actor1, pose1, actor2, pose2)));
  auto drive = mDrives.back().get();
  if (actor1) {
    actor1->addDrive(drive);
    if (actor1->getType() == EActorType::DYNAMIC) {
      static_cast<PxRigidDynamic *>(actor1->getPxActor())->wakeUp();
    } else if (actor1->getType() == EActorType::ARTICULATION_LINK) {
      static_cast<PxArticulationLink *>(actor1->getPxActor())->getArticulation().wakeUp();
    }
  }
  if (actor2) {
    actor2->addDrive(drive);
    if (actor2->getType() == EActorType::DYNAMIC) {
      static_cast<PxRigidDynamic *>(actor2->getPxActor())->wakeUp();
    } else if (actor2->getType() == EActorType::ARTICULATION_LINK) {
      static_cast<PxArticulationLink *>(actor2->getPxActor())->getArticulation().wakeUp();
    }
  }
  return static_cast<SDrive6D *>(drive);
}

void SScene::addActor(std::unique_ptr<SActorBase> actor) {
  mPxScene->addActor(*actor->getPxActor());
  mActorId2Actor[actor->getId()] = actor.get();
  mActors.push_back(std::move(actor));
}

void SScene::addArticulation(std::unique_ptr<SArticulation> articulation) {
  for (auto link : articulation->getBaseLinks()) {
    mActorId2Link[link->getId()] = link;
  }
  mPxScene->addArticulation(*articulation->getPxArticulation());
  mArticulations.push_back(std::move(articulation));
}

void SScene::addKinematicArticulation(std::unique_ptr<SKArticulation> articulation) {
  for (auto link : articulation->getBaseLinks()) {
    mActorId2Link[link->getId()] = link;
    mPxScene->addActor(*link->getPxActor());
  }
  mKinematicArticulations.push_back(std::move(articulation));
}

void SScene::removeCleanUp1() {
  // advance the destroyed stage to 2

  if (mRequiresRemoveCleanUp1) {
    mRequiresRemoveCleanUp1 = false;
    mRequiresRemoveCleanUp2 = true;
    // release actors
    for (auto &a : mActors) {
      if (a->getDestroyedState() == 1) {
        mPxScene->removeActor(*a->getPxActor());
        a->setDestroyedState(2);
      }
    }

    // release articulation
    for (auto &a : mArticulations) {
      if (a->getDestroyedState() == 1) {
        mPxScene->removeArticulation(*a->getPxArticulation());
        a->setDestroyedState(2);
      }
    }

    // release kinematic articulation
    for (auto &a : mKinematicArticulations) {
      if (a->getDestroyedState() == 1) {
        for (auto l : a->getBaseLinks()) {
          mPxScene->removeActor(*l->getPxActor());
          l->setDestroyedState(2);
        }
      }
    }
  }
}

void SScene::removeCleanUp2() {
  if (mRequiresRemoveCleanUp2) {
    mRequiresRemoveCleanUp2 = false;
    // release actors
    for (auto &a : mActors) {
      if (a->getDestroyedState() == 2) {
        a->getPxActor()->release();
      }
    }
    mActors.erase(std::remove_if(mActors.begin(), mActors.end(),
                                 [](auto &a) { return a->getDestroyedState() == 2; }),
                  mActors.end());

    // release articulation
    for (auto &a : mArticulations) {
      if (a->getDestroyedState() == 2) {
        a->getPxArticulation()->release();
      }
    }
    mArticulations.erase(std::remove_if(mArticulations.begin(), mArticulations.end(),
                                        [](auto &a) { return a->getDestroyedState() == 2; }),
                         mArticulations.end());

    // release kinematic articulation
    for (auto &a : mKinematicArticulations) {
      if (a->getDestroyedState() == 2) {
        for (auto l : a->getBaseLinks()) {
          l->getPxActor()->release();
        }
      }
    }
    mKinematicArticulations.erase(
        std::remove_if(mKinematicArticulations.begin(), mKinematicArticulations.end(),
                       [](auto &a) { return a->getDestroyedState() == 2; }),
        mKinematicArticulations.end());
  }
}

void SScene::removeActor(SActorBase *actor) {
  if (actor->isBeingDestroyed()) {
    return;
  }
  mRequiresRemoveCleanUp1 = true;
  // predestroy event
  EventActorPreDestroy e;
  e.actor = actor;
  actor->EventEmitter<EventActorPreDestroy>::emit(e);

  mActorId2Actor.erase(actor->getId());

  // remove drives
  for (auto drive : actor->getDrives()) {
    removeDrive(drive);
  }

  // remove camera
  removeMountedCameraByMount(actor);

  // remove render bodies
  for (auto body : actor->getRenderBodies()) {
    body->destroy();
  }

  // remove collision bodies
  for (auto body : actor->getCollisionBodies()) {
    body->destroy();
  }

  actor->markDestroyed();
}

void SScene::removeArticulation(SArticulation *articulation) {
  if (articulation->isBeingDestroyed()) {
    return;
  }
  mRequiresRemoveCleanUp1 = true;

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
      removeDrive(drive);
    }

    // remove camera
    removeMountedCameraByMount(link);

    // remove render bodies
    for (auto body : link->getRenderBodies()) {
      body->destroy();
    }

    // remove collision bodies
    for (auto body : link->getCollisionBodies()) {
      body->destroy();
    }

    // remove reference
    mActorId2Link.erase(link->getId());
  }

  // mark removed
  articulation->markDestroyed();
}

void SScene::removeKinematicArticulation(SKArticulation *articulation) {
  if (articulation->isBeingDestroyed()) {
    return;
  }
  mRequiresRemoveCleanUp1 = true;

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
      removeDrive(drive);
    }

    // remove camera
    removeMountedCameraByMount(link);

    // remove render bodies
    for (auto body : link->getRenderBodies()) {
      body->destroy();
    }

    for (auto body : link->getCollisionBodies()) {
      body->destroy();
    }

    // remove reference
    mActorId2Link.erase(link->getId());

    // remove actor
    mPxScene->removeActor(*link->getPxActor());
  }

  articulation->markDestroyed();
  // mKinematicArticulations.erase(
  //     std::remove_if(mKinematicArticulations.begin(), mKinematicArticulations.end(),
  //                    [articulation](auto &a) { return a.get() == articulation; }),
  //     mKinematicArticulations.end());
}

void SScene::removeDrive(SDrive *drive) {
  if (drive->mScene != this) {
    spdlog::get("SAPIEN")->error("Failed to remove drive: drive is not in this scene.");
  }
  drive->getPxJoint()->release();
  if (drive->mActor1) {
    drive->mActor1->removeDrive(drive);
    if (drive->mActor1->getType() == EActorType::DYNAMIC) {
      static_cast<PxRigidDynamic *>(drive->getActor1()->getPxActor())->wakeUp();
    } else if (drive->mActor1->getType() == EActorType::ARTICULATION_LINK) {
      static_cast<PxArticulationLink *>(drive->getActor1()->getPxActor())
          ->getArticulation()
          .wakeUp();
    }
  }
  if (drive->mActor2) {
    drive->mActor2->removeDrive(drive);
    if (drive->mActor2->getType() == EActorType::DYNAMIC) {
      static_cast<PxRigidDynamic *>(drive->getActor2()->getPxActor())->wakeUp();
    } else if (drive->mActor2->getType() == EActorType::ARTICULATION_LINK) {
      static_cast<PxArticulationLink *>(drive->getActor2()->getPxActor())
          ->getArticulation()
          .wakeUp();
    }
  }
  mDrives.erase(std::remove_if(mDrives.begin(), mDrives.end(),
                               [drive](auto &d) { return d.get() == drive; }),
                mDrives.end());
}

SActorBase *SScene::findActorById(physx_id_t id) const {
  auto it = mActorId2Actor.find(id);
  if (it == mActorId2Actor.end()) {
    return nullptr;
  }
  return it->second;
}

SLinkBase *SScene::findArticulationLinkById(physx_id_t id) const {
  auto it = mActorId2Link.find(id);
  if (it == mActorId2Link.end()) {
    return nullptr;
  }
  return it->second;
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

// void SScene::setShadowLight(PxVec3 const &direction, PxVec3 const &color) {
//   if (!mRendererScene) {
//     spdlog::get("SAPIEN")->error("Failed to add light: renderer is not added to simulation.");
//     return;
//   }
//   mRendererScene->setShadowLight({direction.x, direction.y, direction.z},
//                                  {color.x, color.y, color.z});
// }
// void SScene::addPointLight(PxVec3 const &position, PxVec3 const &color) {
//   if (!mRendererScene) {
//     spdlog::get("SAPIEN")->error("Failed to add light: renderer is not added to simulation.");
//     return;
//   }
//   mRendererScene->addPointLight({position.x, position.y, position.z}, {color.x, color.y,
//   color.z});
// }
// void SScene::setAmbientLight(PxVec3 const &color) {
//   if (!mRendererScene) {
//     spdlog::get("SAPIEN")->error("Failed to add light: renderer is not added to simulation.");
//     return;
//   }
//   mRendererScene->setAmbientLight({color.x, color.y, color.z});
// }
// void SScene::addDirectionalLight(PxVec3 const &direction, PxVec3 const &color) {
//   if (!mRendererScene) {
//     spdlog::get("SAPIEN")->error("Failed to add light: renderer is not added to simulation.");
//     return;
//   }
//   mRendererScene->addDirectionalLight({direction.x, direction.y, direction.z},
//                                       {color.x, color.y, color.z});
// }

void SScene::step() {
  EASY_BLOCK("Pre-step processing", profiler::colors::Blue);

  for (auto &a : mActors) {
    if (!a->isBeingDestroyed())
      a->prestep();
  }
  for (auto &a : mArticulations) {
    if (!a->isBeingDestroyed())
      a->prestep();
  }
  for (auto &a : mKinematicArticulations) {
    if (!a->isBeingDestroyed())
      a->prestep();
  }

  // confirm removal of marked objects
  removeCleanUp1();

  EASY_END_BLOCK;
  EASY_BLOCK("PhysX scene Step", profiler::colors::Red);

  mPxScene->simulate(mTimestep);
  while (!mPxScene->fetchResults(true)) {
    // contact callback can happen here
    // the callbacks may remove objects, which are not actually removed in this step
  }

  EASY_END_BLOCK;

  // do removal of marked objects
  removeCleanUp2();

  EventSceneStep event;
  event.scene = this;
  event.timeStep = getTimestep();
  emit(event);
}

void SScene::stepAsync() {
  for (auto &a : mActors) {
    if (!a->isBeingDestroyed())
      a->prestep();
  }
  for (auto &a : mArticulations) {
    if (!a->isBeingDestroyed())
      a->prestep();
  }
  for (auto &a : mKinematicArticulations) {
    if (!a->isBeingDestroyed())
      a->prestep();
  }
  removeCleanUp1();
  mPxScene->simulate(mTimestep);
}

void SScene::stepWait() {
  while (!mPxScene->fetchResults(true)) {
  }

  removeCleanUp2();

  EventSceneStep event;
  event.scene = this;
  event.timeStep = getTimestep();
  emit(event);
}

void SScene::updateRender() {
  EASY_FUNCTION("Update Render", profiler::colors::Magenta);

  if (!mRendererScene) {
    spdlog::get("SAPIEN")->error("Failed to update render: renderer is not added.");
    return;
  }
  for (auto &actor : mActors) {
    if (!actor->isBeingDestroyed()) {
      actor->updateRender(actor->getPxActor()->getGlobalPose());
    }
  }

  for (auto &articulation : mArticulations) {
    for (auto &link : articulation->getBaseLinks()) {
      if (!articulation->isBeingDestroyed()) {
        link->updateRender(link->getPxActor()->getGlobalPose());
      }
    }
  }

  for (auto &articulation : mKinematicArticulations) {
    for (auto &link : articulation->getBaseLinks()) {
      if (!articulation->isBeingDestroyed()) {
        link->updateRender(link->getPxActor()->getGlobalPose());
      }
    }
  }

  for (auto &cam : mCameras) {
    cam.camera->setPose(cam.actor->getPxActor()->getGlobalPose());
  }

  getRendererScene()->updateRender();
}

SActorStatic *SScene::addGround(PxReal altitude, bool render,
                                std::shared_ptr<SPhysicalMaterial> material,
                                std::shared_ptr<Renderer::IPxrMaterial> renderMaterial) {
  return createActorBuilder()->buildGround(altitude, render, material, renderMaterial, "ground");
}

void SScene::updateContact(std::unique_ptr<SContact> contact) {
  auto pair = std::make_pair(contact->collisionShapes[0]->getPxShape(),
                             contact->collisionShapes[1]->getPxShape());
  if (contact->starts) {
    // NOTE: contact actually can start twice
    mContacts[pair] = std::move(contact);
  } else if (contact->persists) {
    auto it = mContacts.find(pair);
    if (it == mContacts.end()) {
      spdlog::get("SAPIEN")->error("Error updating contact pair: it has not started");
    }
    it->second = std::move(contact);
  } else if (contact->ends) {
    auto it = mContacts.find(pair);
    if (it == mContacts.end()) {
      spdlog::get("SAPIEN")->error("Error ending contact pair: it has not started");
      return;
    }
    mContacts.erase(it);
  }
}

std::vector<SContact *> SScene::getContacts() const {
  std::vector<SContact *> contacts{};
  for (auto &it : mContacts) {
    contacts.push_back(it.second.get());
  }
  return contacts;
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

std::vector<SLight *> SScene::getAllLights() const {
  std::vector<SLight *> output;
  for (auto &light : mLights) {
    output.push_back(light.get());
  }
  return output;
}

std::map<physx_id_t, std::string> SScene::findRenderId2VisualName() const {
  std::map<physx_id_t, std::string> result;
  for (auto &actor : mActors) {
    for (auto &v : actor->getRenderBodies()) {
      result[v->getUniqueId()] = v->getName();
    }
  }
  for (auto &articulation : mArticulations) {
    for (auto &actor : articulation->getBaseLinks()) {
      for (auto &v : actor->getRenderBodies()) {
        result[v->getUniqueId()] = v->getName();
      }
    }
  }
  for (auto &articulation : mKinematicArticulations) {
    for (auto &actor : articulation->getBaseLinks()) {
      for (auto &v : actor->getRenderBodies()) {
        result[v->getUniqueId()] = v->getName();
      }
    }
  }
  return result;
}

SceneData SScene::packScene() {
  SceneData data;
  for (auto &actor : mActors) {
    data.mActorData[actor->getId()] = actor->packData();
  }
  for (auto &articulation : mArticulations) {
    data.mArticulationData[articulation->getRootLink()->getId()] = articulation->packData();
    data.mArticulationDriveData[articulation->getRootLink()->getId()] = articulation->packDrive();
  }
  for (auto &articulation : mKinematicArticulations) {
    for (auto actor : articulation->getBaseLinks()) {
      data.mActorData[actor->getId()] = actor->packData();
    }
  }
  return data;
}

void SScene::unpackScene(SceneData const &data) {
  for (auto &actor : mActors) {
    auto it = data.mActorData.find(actor->getId());
    if (it != data.mActorData.end()) {
      actor->unpackData(it->second);
    }
  }
  for (auto &articulation : mArticulations) {
    {
      auto it = data.mArticulationData.find(articulation->getRootLink()->getId());
      if (it != data.mArticulationData.end()) {
        articulation->unpackData(it->second);
      }
    }
    {
      auto it = data.mArticulationDriveData.find(articulation->getRootLink()->getId());
      if (it != data.mArticulationDriveData.end()) {
        articulation->unpackDrive(it->second);
      }
    }
  }
  for (auto &articulation : mKinematicArticulations) {
    for (auto actor : articulation->getBaseLinks()) {
      auto it = data.mActorData.find(actor->getId());
      if (it != data.mActorData.end()) {
        actor->unpackData(it->second);
      }
    }
  }
}

void SScene::setAmbientLight(PxVec3 const &color) {
  mRendererScene->setAmbientLight({color.x, color.y, color.z});
}

PxVec3 SScene::getAmbientLight() const {
  auto light = mRendererScene->getAmbientLight();
  return {light[0], light[1], light[2]};
}

SPointLight *SScene::addPointLight(PxVec3 const &position, PxVec3 const &color, bool enableShadow,
                                   float shadowNear, float shadowFar) {
  auto light = mRendererScene->addPointLight({position.x, position.y, position.z},
                                             {color.x, color.y, color.z}, enableShadow, shadowNear,
                                             shadowFar);
  auto sl = std::make_unique<SPointLight>(this, light);
  auto ret = sl.get();
  mLights.push_back(std::move(sl));
  return ret;
}

SDirectionalLight *SScene::addDirectionalLight(PxVec3 const &direction, PxVec3 const &color,
                                               bool enableShadow, PxVec3 const &position,
                                               float shadowScale, float shadowNear,
                                               float shadowFar) {
  auto light = mRendererScene->addDirectionalLight(
      {direction.x, direction.y, direction.z}, {color.x, color.y, color.z}, enableShadow,
      {position.x, position.y, position.z}, shadowScale, shadowNear, shadowFar);
  auto sl = std::make_unique<SDirectionalLight>(this, light);
  auto ret = sl.get();
  mLights.push_back(std::move(sl));
  return ret;
}

SSpotLight *SScene::addSpotLight(PxVec3 const &position, PxVec3 const &direction, float fovInner,
                                 float fovOuter, PxVec3 const &color, bool enableShadow,
                                 float shadowNear, float shadowFar) {
  auto light = mRendererScene->addSpotLight(
      {position.x, position.y, position.z}, {direction.x, direction.y, direction.z}, fovInner,
      fovOuter, {color.x, color.y, color.z}, enableShadow, shadowNear, shadowFar);
  auto sl = std::make_unique<SSpotLight>(this, light);
  auto ret = sl.get();
  mLights.push_back(std::move(sl));
  return ret;
}

void SScene::removeLight(SLight *light) {
  if (light && light->getRendererLight()) {
    mRendererScene->removeLight(light->getRendererLight());
  }
  mLights.erase(
      std::remove_if(mLights.begin(), mLights.end(), [=](auto &l) { return l.get() == light; }),
      mLights.end());
}

}; // namespace sapien
