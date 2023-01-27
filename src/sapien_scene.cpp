#include "sapien/sapien_scene.h"
#include "sapien/actor_builder.h"
#include "sapien/articulation/articulation_builder.h"
#include "sapien/articulation/sapien_articulation.h"
#include "sapien/articulation/sapien_joint.h"
#include "sapien/articulation/sapien_kinematic_articulation.h"
#include "sapien/articulation/sapien_kinematic_joint.h"
#include "sapien/articulation/sapien_link.h"
#include "sapien/articulation/urdf_loader.h"
#include "sapien/filter_shader.h"
#include "sapien/renderer/render_interface.h"
#include "sapien/sapien_actor.h"
#include "sapien/sapien_contact.h"
#include "sapien/sapien_drive.h"
#include "sapien/sapien_entity_particle.h"
#include "sapien/sapien_gear.h"
#include "sapien/simulation.h"
#include <algorithm>
#include <spdlog/spdlog.h>

#include <easy/profiler.h>

namespace sapien {

/************************************************
 * Basic
 ***********************************************/
SScene::SScene(std::shared_ptr<Simulation> sim, SceneConfig const &config)
    : mSimulationShared(sim), mSimulationCallback(this), mRendererScene(nullptr) {

  PxSceneDesc sceneDesc(sim->mPhysicsSDK->getTolerancesScale());
  sceneDesc.gravity = PxVec3({config.gravity.x(), config.gravity.y(), config.gravity.z()});
  sceneDesc.filterShader = TypeAffinityIgnoreFilterShader;
  sceneDesc.solverType = config.enableTGS ? PxSolverType::eTGS : PxSolverType::ePGS;
  sceneDesc.bounceThresholdVelocity = config.bounceThreshold;

  PxSceneFlags sceneFlags;
  if (config.enableEnhancedDeterminism) {
    sceneFlags |= PxSceneFlag::eENABLE_ENHANCED_DETERMINISM;
  }
  if (config.enablePCM) {
    sceneFlags |= PxSceneFlag::eENABLE_PCM;
  }
  if (config.enableCCD) {
    sceneFlags |= PxSceneFlag::eENABLE_CCD;
  }
  if (config.enableFrictionEveryIteration) {
    sceneFlags |= PxSceneFlag::eENABLE_FRICTION_EVERY_ITERATION;
  }
  if (config.enableAdaptiveForce) {
    sceneFlags |= PxSceneFlag::eADAPTIVE_FORCE;
  }
  sceneDesc.flags = sceneFlags;

  mCpuDispatcher = PxDefaultCpuDispatcherCreate(0);
  if (!mCpuDispatcher) {
    spdlog::get("SAPIEN")->critical("Failed to create PhysX CPU dispatcher");
    throw std::runtime_error("Scene Creation Failed");
  }
  sceneDesc.cpuDispatcher = mCpuDispatcher;

  mPxScene = mSimulationShared->mPhysicsSDK->createScene(sceneDesc);

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
    mRendererScene = renderer->createScene(""); // FIXME: pass scene name here
  }
  mDisableCollisionVisual = config.disableCollisionVisual;
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
    drive->getPxJoint()->release();
  }
  for (auto &gear : mGears) {
    gear->mGearJoint->release();
  }
  mPxScene->release();

  // TODO: check whether we implement mXXX.release() to replace the workaround
  mActors.clear();
  mArticulations.clear();
  mKinematicArticulations.clear();

  if (mRendererScene) {
    mSimulationShared->getRenderer()->removeScene(mRendererScene);
  }

  mCpuDispatcher->release();
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
  wakeUpActor(actor1);
  wakeUpActor(actor2);
  return static_cast<SDrive6D *>(drive);
}

SGear *SScene::createGear(SActorDynamicBase *actor1, PxTransform const &pose1,
                          SActorDynamicBase *actor2, PxTransform const &pose2) {
  mGears.push_back(std::make_unique<SGear>(this, actor1, pose1, actor2, pose2));
  return mGears.back().get();
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

void SScene::removeCleanUp() {
  // advance the destroyed stage to 2

  if (mRequiresRemoveCleanUp) {
    mRequiresRemoveCleanUp = false;

    // clear contacts
    std::erase_if(mContacts, [](const auto &item) {
      auto const &[key, value] = item;
      return value->actors[0]->isBeingDestroyed() || value->actors[1]->isBeingDestroyed();
    });

    // release actors
    for (auto &a : mActors) {
      if (a->getDestroyedState() == 1) {
        a->getPxActor()->userData = nullptr;
        mPxScene->removeActor(*a->getPxActor());
        // a->setDestroyedState(2);
        a->getPxActor()->release();
      }
    }

    // release articulation
    for (auto &a : mArticulations) {
      if (a->getDestroyedState() == 1) {

        for (auto l : a->getSLinks()) {
          l->getPxActor()->userData = nullptr;
        }
        a->getPxArticulation()->userData = nullptr;

        mPxScene->removeArticulation(*a->getPxArticulation());
        // a->setDestroyedState(2);

        a->getPxArticulation()->release();
      }
    }

    // release kinematic articulation
    for (auto &a : mKinematicArticulations) {
      if (a->getDestroyedState() == 1) {
        for (auto l : a->getBaseLinks()) {
          l->getPxActor()->userData = nullptr;
          mPxScene->removeActor(*l->getPxActor());
          // l->setDestroyedState(2);

          l->getPxActor()->release();
        }
      }
    }

    mActors.erase(std::remove_if(mActors.begin(), mActors.end(),
                                 [](auto &a) { return a->isBeingDestroyed(); }),
                  mActors.end());
    mArticulations.erase(std::remove_if(mArticulations.begin(), mArticulations.end(),
                                        [](auto &a) { return a->isBeingDestroyed(); }),
                         mArticulations.end());
    mKinematicArticulations.erase(std::remove_if(mKinematicArticulations.begin(),
                                                 mKinematicArticulations.end(),
                                                 [](auto &a) { return a->isBeingDestroyed(); }),
                                  mKinematicArticulations.end());
  }
}

void SScene::removeActor(SActorBase *actor) {
  if (actor->isBeingDestroyed()) {
    return;
  }
  mRequiresRemoveCleanUp = true;
  // predestroy event
  EventActorPreDestroy e;
  e.actor = actor;
  actor->EventEmitter<EventActorPreDestroy>::emit(e);

  mActorId2Actor.erase(actor->getId());

  // remove drives
  for (auto it = mDrives.begin(); it != mDrives.end();) {
    if ((*it)->getActor1() == actor || (*it)->getActor2() == actor) {
      wakeUpActor((*it)->getActor1());
      wakeUpActor((*it)->getActor2());
      (*it)->getPxJoint()->release();
      it = mDrives.erase(it);
    } else {
      ++it;
    }
  }

  for (auto it = mGears.begin(); it != mGears.end();) {
    if ((*it)->getActor1() == actor || (*it)->getActor2() == actor) {
      wakeUpActor((*it)->getActor1());
      wakeUpActor((*it)->getActor2());
      (*it)->getGearJoint()->release();
      it = mGears.erase(it);
    } else {
      ++it;
    }
  }

  // remove camera
  removeCameraByParent(actor);

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
  mRequiresRemoveCleanUp = true;

  EventArticulationPreDestroy e;
  e.articulation = articulation;
  articulation->EventEmitter<EventArticulationPreDestroy>::emit(e);

  for (auto link : articulation->getBaseLinks()) {
    // predestroy event
    EventActorPreDestroy e;
    e.actor = link;
    link->EventEmitter<EventActorPreDestroy>::emit(e);

    // remove drives
    for (auto it = mDrives.begin(); it != mDrives.end();) {
      if ((*it)->getActor1() == link || (*it)->getActor2() == link) {
        wakeUpActor((*it)->getActor1());
        wakeUpActor((*it)->getActor2());
        (*it)->getPxJoint()->release();
        it = mDrives.erase(it);
      } else {
        ++it;
      }
    }
    for (auto it = mGears.begin(); it != mGears.end();) {
      if ((*it)->getActor1() == link || (*it)->getActor2() == link) {
        wakeUpActor((*it)->getActor1());
        wakeUpActor((*it)->getActor2());
        (*it)->getGearJoint()->release();
        it = mGears.erase(it);
      } else {
        ++it;
      }
    }

    // remove camera
    removeCameraByParent(link);

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
  mRequiresRemoveCleanUp = true;

  EventArticulationPreDestroy e;
  e.articulation = articulation;
  articulation->EventEmitter<EventArticulationPreDestroy>::emit(e);

  for (auto link : articulation->getBaseLinks()) {
    // predestroy event
    EventActorPreDestroy e;
    e.actor = link;
    link->EventEmitter<EventActorPreDestroy>::emit(e);

    // remove drives
    for (auto it = mDrives.begin(); it != mDrives.end();) {
      if ((*it)->getActor1() == link || (*it)->getActor2() == link) {
        wakeUpActor((*it)->getActor1());
        wakeUpActor((*it)->getActor2());
        (*it)->getPxJoint()->release();
        it = mDrives.erase(it);
      } else {
        ++it;
      }
    }

    for (auto it = mGears.begin(); it != mGears.end();) {
      if ((*it)->getActor1() == link || (*it)->getActor2() == link) {
        wakeUpActor((*it)->getActor1());
        wakeUpActor((*it)->getActor2());
        (*it)->getGearJoint()->release();
        it = mGears.erase(it);
      } else {
        ++it;
      }
    }

    // remove camera
    removeCameraByParent(link);

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
}

void SScene::removeDrive(SDrive *drive) {
  if (drive->mScene != this) {
    spdlog::get("SAPIEN")->error("Failed to remove drive: drive is not in this scene.");
  }

  wakeUpActor(drive->getActor1());
  wakeUpActor(drive->getActor2());
  drive->getPxJoint()->release();
  std::erase_if(mDrives, [drive](auto &d) { return d.get() == drive; });
}

void SScene::removeGear(SGear *gear) {
  if (gear->mScene != this) {
    spdlog::get("SAPIEN")->error("Failed to remove gear: gear is not in this scene.");
  }
  wakeUpActor(gear->getActor1());
  wakeUpActor(gear->getActor2());
  gear->getGearJoint()->release();
  std::erase_if(mGears, [=](auto const &g) { return g.get() == gear; });
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

void SScene::wakeUpActor(SActorBase *actor) {
  if (auto a = dynamic_cast<SActor *>(actor)) {
    a->getPxActor()->wakeUp();
    return;
  }
  if (auto a = dynamic_cast<SLink *>(actor)) {
    a->getArticulation()->getPxArticulation()->wakeUp();
    return;
  }
}

std::vector<SCamera *> SScene::getCameras() {
  std::vector<SCamera *> cameras;
  cameras.reserve(mCameras.size());
  for (auto &cam : mCameras) {
    cameras.push_back(cam.get());
  }
  return cameras;
}

SCamera *SScene::addCamera(std::string const &name, uint32_t width, uint32_t height, float fovy,
                           float near, float far) {
  if (!mRendererScene) {
    spdlog::get("SAPIEN")->error("Failed to add camera: renderer is not added to simulation.");
    return nullptr;
  }
  auto cam = std::make_unique<SCamera>(this, width, height, fovy, near, far);
  cam->setName(name);
  // cam->setFovY(fovy, true);
  // cam->setNear(near);
  // cam->setFar(far);
  mCameras.push_back(std::move(cam));
  return mCameras.back().get();
}

void SScene::removeCamera(SCamera *cam) {
  if (mRendererScene) {
    mRendererScene->removeCamera(cam->getRendererCamera());
  }
  mCameras.erase(std::remove_if(mCameras.begin(), mCameras.end(),
                                [cam](std::unique_ptr<SCamera> &mc) { return mc.get() == cam; }),
                 mCameras.end());
}

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
  removeCleanUp();

  EASY_END_BLOCK;
  EASY_BLOCK("PhysX scene Step", profiler::colors::Red);

  mPxScene->simulate(mTimestep);
  while (!mPxScene->fetchResults(true)) {
    // contact callback can happen here
    // the callbacks may remove objects, which are not actually removed in this step
  }

  EASY_END_BLOCK;

  EventSceneStep event;
  event.scene = this;
  event.timeStep = getTimestep();
  emit(event);
}

std::future<void> SScene::stepAsync() {
  return getThread().submit([this]() {
    EASY_BLOCK("Scene preprocess")
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
    removeCleanUp();
    EASY_END_BLOCK

    EASY_BLOCK("PhysX scene simulate", profiler::colors::Red);
    mPxScene->simulate(mTimestep);
    EASY_END_BLOCK

    EASY_BLOCK("PhysX scene fetch", profiler::colors::Red);
    while (!mPxScene->fetchResults(true)) {
    }
    EASY_END_BLOCK

    EASY_BLOCK("Scene postprocess");
    // removeCleanUp2();

    EventSceneStep event;
    event.scene = this;
    event.timeStep = getTimestep();
    emit(event);
    EASY_END_BLOCK
  });
}

std::future<void> SScene::multistepAsync(int steps, SceneMultistepCallback *callback) {
  return getThread().submit([this, steps, callback]() {
    {
      EASY_BLOCK("BeforeMultistep")
      callback->beforeMultistep();
    }

    for (int s = 0; s < steps; ++s) {
      {
        EASY_BLOCK("BeforeStep")
        callback->beforeStep(s);
      }

      {
        EASY_BLOCK("Scene preprocess")
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
        removeCleanUp();
      }

      {
        EASY_BLOCK("PhysX scene simulate", profiler::colors::Red);
        mPxScene->simulate(mTimestep);
      }

      {
        EASY_BLOCK("PhysX scene fetch", profiler::colors::Red);
        while (!mPxScene->fetchResults(true)) {
        }
      }

      {
        EASY_BLOCK("AfterStep")
        callback->afterStep(s);
      }
    }
    {
      EASY_BLOCK("AfterMultistep")
      callback->afterMultistep();
    }

    EventSceneStep event;
    event.scene = this;
    event.timeStep = getTimestep();
    emit(event);
    EASY_END_BLOCK
  });
}

// void SScene::stepWait() {
//   // while (!mPxScene->fetchResults(true)) {
//   // }
//   mStep.get();
// }

void SScene::updateRender() {
  EASY_FUNCTION("Update Render", profiler::colors::Magenta);
  std::lock_guard lock(mUpdateRenderMutex);

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
    cam->update();
  }

  for (auto &light : mLights) {
    light->update();
  }

  getRendererScene()->updateRender();
}

void SScene::updateRenderAndTakePictures(std::vector<SCamera *> const &cameras) {
  std::lock_guard lock(mUpdateRenderMutex);

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
    cam->update();
  }

  std::vector<Renderer::ICamera *> rcams;
  for (auto cam : cameras) {
    rcams.push_back(cam->getRendererCamera());
  }
  getRendererScene()->updateRenderAndTakePictures(rcams);
}

std::future<void> SScene::updateRenderAsync() {
  return getThread().submit([this]() { updateRender(); });
}

SActorStatic *SScene::addGround(PxReal altitude, bool render,
                                std::shared_ptr<SPhysicalMaterial> material,
                                std::shared_ptr<Renderer::IPxrMaterial> renderMaterial,
                                PxVec2 const &renderSize) {
  return createActorBuilder()->buildGround(altitude, render, material, renderMaterial, renderSize,
                                           "ground");
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
                                   float shadowNear, float shadowFar, uint32_t shadowMapSize) {
  auto light = mRendererScene->addPointLight({0, 0, 0}, {color.x, color.y, color.z}, enableShadow,
                                             shadowNear, shadowFar, shadowMapSize);
  auto sl = std::make_unique<SPointLight>(this, light);
  sl->setPosition(position);
  auto ret = sl.get();
  mLights.push_back(std::move(sl));
  return ret;
}

SDirectionalLight *SScene::addDirectionalLight(PxVec3 const &direction, PxVec3 const &color,
                                               bool enableShadow, PxVec3 const &position,
                                               float shadowScale, float shadowNear,
                                               float shadowFar, uint32_t shadowMapSize) {
  auto light = mRendererScene->addDirectionalLight({1, 0, 0}, {color.x, color.y, color.z},
                                                   enableShadow, {0, 0, 0}, shadowScale,
                                                   shadowNear, shadowFar, shadowMapSize);
  auto sl = std::make_unique<SDirectionalLight>(this, light);
  sl->setPosition(position);
  sl->setDirection(direction);
  auto ret = sl.get();
  mLights.push_back(std::move(sl));
  return ret;
}

SSpotLight *SScene::addSpotLight(PxVec3 const &position, PxVec3 const &direction, float fovInner,
                                 float fovOuter, PxVec3 const &color, bool enableShadow,
                                 float shadowNear, float shadowFar, uint32_t shadowMapSize) {
  auto light = mRendererScene->addSpotLight({0, 0, 0}, {1, 0, 0}, fovInner, fovOuter,
                                            {color.x, color.y, color.z}, enableShadow, shadowNear,
                                            shadowFar, shadowMapSize);
  auto sl = std::make_unique<SSpotLight>(this, light);
  sl->setPosition(position);
  sl->setDirection(direction);
  auto ret = sl.get();
  mLights.push_back(std::move(sl));
  return ret;
}

SActiveLight *SScene::addActiveLight(PxTransform const &pose, PxVec3 const &color, float fov,
                                     std::string_view texPath, float shadowNear, float shadowFar,
                                     uint32_t shadowMapSize) {
  auto light = mRendererScene->addActiveLight(PxTransform{PxIdentity}, {color.x, color.y, color.z},
                                              fov, texPath, shadowNear, shadowFar, shadowMapSize);
  auto sl = std::make_unique<SActiveLight>(this, light);
  sl->setPose(pose);
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

void SScene::setEnvironmentMap(std::string_view filename) {
  mRendererScene->setEnvironmentMap(filename);
}

void SScene::setEnvironmentMapFromFiles(std::string_view px, std::string_view nx,
                                        std::string_view py, std::string_view ny,
                                        std::string_view pz, std::string_view nz) {
  mRendererScene->setEnvironmentMap({px, nx, py, ny, pz, nz});
}

void SScene::removeCameraByParent(SActorBase *actor) {
  auto start =
      std::remove_if(mCameras.begin(), mCameras.end(),
                     [actor](std::unique_ptr<SCamera> &mc) { return mc->getParent() == actor; });
  for (auto it = start; it != mCameras.end(); ++it) {
    mRendererScene->removeCamera((*it)->getRendererCamera());
  }
  mCameras.erase(start, mCameras.end());
}

SEntityParticle *SScene::addParticleEntity(
    Eigen::Ref<Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>> positions) {
  auto body = mRendererScene->addPointBody(positions);
  mParticlesEntities.push_back(std::make_unique<SEntityParticle>(this, body));
  return mParticlesEntities.back().get();
}

void SScene::removeParticleEntity(SEntityParticle *entity) {
  auto start =
      std::remove_if(mParticlesEntities.begin(), mParticlesEntities.end(),
                     [=](std::unique_ptr<SEntityParticle> &e) { return e.get() == entity; });
  for (auto it = start; it != mParticlesEntities.end(); ++it) {
    mRendererScene->removePointBody(entity->getVisualBody());
  }
  mParticlesEntities.erase(start, mParticlesEntities.end());
}

ThreadPool &SScene::getThread() {
  if (!mRunnerThread.running()) {
    mRunnerThread.init();
  }
  return mRunnerThread;
}
}; // namespace sapien
