#include "sapien/articulation/articulation_builder.h"
#include "sapien/articulation/sapien_articulation.h"
#include "sapien/articulation/sapien_link.h"
#include "sapien/gear_joint.h"
#include "sapien/renderer/svulkan2_renderer.h"
#include "sapien/renderer/svulkan2_rigidbody.h"
#include "sapien/renderer/svulkan2_window.h"
#include "sapien/sapien_actor.h"
#include "sapien/sapien_drive.h"
#include "sapien/sapien_gear.h"
#include "sapien/sapien_scene.h"
#include "sapien/simulation.h"
#include <iostream>

using namespace sapien;

int main() {
  Renderer::SVulkan2Renderer::setLogLevel("info");
  auto sim = std::make_shared<Simulation>();
  auto renderer =
      std::make_shared<Renderer::SVulkan2Renderer>(false, 1000, 1000, 4, "", "back", false);
  Renderer::setDefaultCameraShaderDirectory("../vulkan_shader/ibl");
  sim->setRenderer(renderer);
  Renderer::SVulkan2Window window(renderer, 800, 600, "../vulkan_shader/ibl");

  SceneConfig config;
  config.gravity = {0, 0, 0};

  auto scene = sim->createScene(config);

  scene->addGround(0);
  scene->getRendererScene()->setAmbientLight({0.3, 0.3, 0.3});
  scene->getRendererScene()->addDirectionalLight({0, 1, -1}, {0.5, 0.5, 0.5}, true, {0, 0, 0}, 10,
                                                 -10, 10, 1024);

  auto builder = scene->createActorBuilder();
  builder->addBoxShape(PxTransform({0, 0, 0}, PxIdentity), {0.05, 0.2, 0.2});
  builder->addBoxVisual(PxTransform({0, 0, 0}, PxIdentity), {0.05, 0.2, 0.2});
  auto box1 = builder->build();
  box1->setPose({{0, 0, 0.5}, PxIdentity});
  box1->setName("box1");

  auto box2 = builder->build();
  box2->setPose({{0, 0, 1.5}, PxIdentity});
  box2->setName("box2");

  scene->updateRender();
  scene->step();

  // auto joint = PxRevoluteJointCreate(*sim->mPhysicsSDK, box1->getPxActor(), {{0, 0, 0},
  // PxIdentity},
  //                       box2->getPxActor(), {{0, 0, 0}, PxIdentity});
  // joint->setProjectionLinearTolerance(0.0);
  // joint->setProjectionAngularTolerance(0.0);
  // joint->setDriveForceLimit(PX_MAX_F32);
  // joint->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, true);
  // joint->setDriveVelocity(0.0);
  // joint->setDriveGearRatio(-1.0);

  SGear *gear = scene->createGear(box1, {{0, 0, 0}, PxIdentity}, box2, {{0, 0, 0}, PxIdentity});
  gear->getGearJoint()->setRatio(-2.0);

  window.setScene(static_cast<Renderer::SVulkan2Scene *>(scene->getRendererScene()));
  window.render("Color");
  window.mCameraController->setXYZ(-4, 0, 0.5);

  int iter = 0;
  while (1) {
    scene->updateRender();
    scene->step();
    iter++;

    box1->addForceTorque({0, 0, 0}, {1, 0, 0});

    window.render("Color");
    if (window.windowCloseRequested()) {
      window.close();
      break;
    }
  }

  return 0;
}
