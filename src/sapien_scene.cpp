#include "sapien_scene.h"
#include "actor_builder.h"
#include "articulation/articulation_builder.h"
#include "articulation/sapien_articulation.h"
#include "articulation/sapien_joint.h"
#include "articulation/sapien_link.h"
#include "articulation/urdf_loader.h"
#include "renderer/render_interface.h"
#include "sapien_actor.h"
#include "sapien_contact.h"
#include "simulation.h"
#include <algorithm>
#include <spdlog/spdlog.h>

namespace sapien {

SScene::SScene(Simulation *sim, PxScene *scene)
    : mSimulation(sim), mPxScene(scene), mRendererScene(nullptr), mSimulationCallback(this) {
  auto renderer = sim->getRenderer();
  if (renderer) {
    spdlog::info("Creating scene in renderer");
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

void SScene::removeActor(SActorBase *actor) {
  mLinkId2Actor.erase(actor->getId());

  // remove camera
  std::remove_if(mCameras.begin(), mCameras.end(),
                 [actor](MountedCamera &mc) { return mc.actor == actor; });

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
  actor->getPxActor()->release(); // FIXME: check if this release can be moved to creation time

  // remove sapien actor
  std::remove_if(mActors.begin(), mActors.end(), [actor](auto &a) { return a.get() == actor; });
}

void SScene::removeArticulation(SArticulation *articulation) {
  for (auto link : articulation->getBaseLinks()) {
    // remove camera
    std::remove_if(mCameras.begin(), mCameras.end(),
                   [link](MountedCamera &mc) { return mc.actor == link; });

    // remove render bodies
    for (auto body : link->getRenderBodies()) {
      mRenderId2VisualName.erase(body->getUniqueId());
      body->destroy();
    }

    // remove reference
    mLinkId2Actor.erase(link->getId());
  }

  // remove physical bodies
  mPxScene->removeArticulation(*articulation->getPxArticulation());
  articulation->getPxArticulation()->release();

  // remove sapien articulation
  std::remove_if(mArticulations.begin(), mArticulations.end(),
                 [articulation](auto &a) { return a.get() == articulation; });
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

Renderer::ICamera *SScene::addMountedCamera(std::string const &name, SActorBase *actor,
                                            PxTransform const &pose, uint32_t width,
                                            uint32_t height, float fovx, float fovy, float near,
                                            float far) {
  if (!mRendererScene) {
    spdlog::error("Failed to add camera: renderer is not added to simulation.");
    return nullptr;
  }
  auto cam = mRendererScene->addCamera(name, width, height, fovx, fovy, near, far);
  cam->setInitialPose({{0, 0, 0}, {-0.5, 0.5, 0.5, -0.5}});
  mCameras.push_back({actor, cam});
  return cam;
}

void SScene::removeMountedCamera(Renderer::ICamera *cam) {
  std::remove_if(mCameras.begin(), mCameras.end(),
                 [cam](MountedCamera &mc) { return mc.camera == cam; });
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
    spdlog::error("Failed to add light: renderer is not added to simulation.");
    return;
  }
  mRendererScene->setShadowLight({direction.x, direction.y, direction.z},
                                 {color.x, color.y, color.z});
}
void SScene::addPointLight(PxVec3 const &position, PxVec3 const &color) {
  if (!mRendererScene) {
    spdlog::error("Failed to add light: renderer is not added to simulation.");
    return;
  }
  mRendererScene->addPointLight({position.x, position.y, position.z}, {color.x, color.y, color.z});
}
void SScene::setAmbientLight(PxVec3 const &color) {
  if (!mRendererScene) {
    spdlog::error("Failed to add light: renderer is not added to simulation.");
    return;
  }
  mRendererScene->setAmbientLight({color.x, color.y, color.z});
}
void SScene::addDirectionalLight(PxVec3 const &direction, PxVec3 const &color) {
  if (!mRendererScene) {
    spdlog::error("Failed to add light: renderer is not added to simulation.");
    return;
  }
  mRendererScene->addDirectionalLight({direction.x, direction.y, direction.z},
                                      {color.x, color.y, color.z});
}

void SScene::step() {
  clearContacts();
  mPxScene->simulate(mTimestep);
  while (!mPxScene->fetchResults(true)) {
  }

  // FIXME: update articulation cache
  // FIXME: process callbacks
}

void SScene::updateRender() {
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

  // FIXME: update other articulation

  for (auto &cam : mCameras) {
    cam.camera->setPose(cam.actor->getPxActor()->getGlobalPose());
  }
}

void SScene::addGround(PxReal altitude, bool render, PxMaterial *material) {
  createActorBuilder()->buildGround(altitude, render, material, "ground");
}

void SScene::addContact(SContact const &contact) { mContacts.push_back(contact); }
void SScene::clearContacts() { mContacts.clear(); }
std::vector<SContact> const &SScene::getContacts() const { return mContacts; }

}; // namespace sapien
