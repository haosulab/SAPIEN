#include "sapien_scene.h"
#include "actor_builder.h"
#include "renderer/render_interface.h"
#include "sapien_actor.h"
#include "simulation.h"
#include <algorithm>
#include <spdlog/spdlog.h>

namespace sapien {

SScene::SScene(Simulation *sim, PxScene *scene)
    : mSimulation(sim), mPxScene(scene), mRendererScene(nullptr) {
  auto renderer = sim->getRenderer();
  if (renderer) {
    spdlog::info("Creating scene in renderer");
    mRendererScene = renderer->createScene();
  }
}

SScene::~SScene() {
  if (mRendererScene) {
    mSimulation->getRenderer()->removeScene(mRendererScene);
  }
  if (mPxScene) {
    mPxScene->release();
  }
}

void SScene::addActor(std::unique_ptr<SActor> actor) {
  mPxScene->addActor(*actor->getPxActor());
  mActors.push_back(std::move(actor));
}

void SScene::removeActor(SActor *actor) {
  // remove camera
  std::remove_if(mCameras.begin(), mCameras.end(),
                 [actor](MountedCamera &mc) { return mc.actor == actor; });

  std::remove_if(mActors.begin(), mActors.end(), [actor](auto &a) { return a.get() == actor; });
  mPxScene->removeActor(*actor->getPxActor());
  for (auto body : actor->getRenderBodies()) {
    body->destroy();
  }
  actor->getPxActor()->release();
}

std::unique_ptr<ActorBuilder> SScene::createActorBuilder() {
  return std::make_unique<ActorBuilder>(this);
}

Renderer::ICamera *SScene::addMountedCamera(std::string const &name, SActor *actor,
                                            PxTransform const &pose, uint32_t width,
                                            uint32_t height, float fovx, float fovy, float near,
                                            float far) {
  auto cam = mRendererScene->addCamera(name, width, height, fovx, fovy, near, far);
  mCameras.push_back({actor, cam});
  return cam;
}

void SScene::removeMountedCamera(Renderer::ICamera *cam) {
  std::remove_if(mCameras.begin(), mCameras.end(),
                 [cam](MountedCamera &mc) { return mc.camera == cam; });
}

void SScene::step() {
  mPxScene->simulate(mTimestep);
  while (!mPxScene->fetchResults(true)) {
  }

  // TODO: update articulation cache
  // TODO: process callbacks
}

void SScene::updateRender() {
  if (!mRendererScene) {
    return;
  }
  for (auto &actor : mActors) {
    actor->updateRender(actor->getPxActor()->getGlobalPose());
  }
  // TODO: update articulation

  for (auto &cam : mCameras) {
    cam.camera->setPose(cam.actor->getPxActor()->getGlobalPose());
  }
}

void SScene::addGround(PxReal altitude, bool render, PxMaterial *material) {
  material = material ? material : mSimulation->mDefaultMaterial;
  auto ground =
      PxCreatePlane(*mSimulation->mPhysicsSDK, PxPlane(0.f, 0.f, 1.f, -altitude), *material);
  mPxScene->addActor(*ground);

  if (render && mRendererScene) {
    auto body = mRendererScene->addRigidbody(PxGeometryType::ePLANE, {10, 10, 10}, {1, 1, 1});
    body->setInitialPose(PxTransform({0, 0, altitude}, PxIdentity));
  }
}

}; // namespace sapien
