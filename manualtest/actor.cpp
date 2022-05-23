#include "sapien/articulation/articulation_builder.h"
#include "sapien/articulation/sapien_articulation.h"
#include "sapien/articulation/sapien_link.h"
#include "sapien/renderer/svulkan2_renderer.h"
#include "sapien/renderer/svulkan2_rigidbody.h"
#include "sapien/renderer/svulkan2_window.h"
#include "sapien/sapien_actor.h"
#include "sapien/sapien_drive.h"
#include "sapien/sapien_scene.h"
#include "sapien/simulation.h"
#include <iostream>

#define PI (3.141592653589793238462643383279502884f)

using namespace sapien;

std::shared_ptr<ArticulationBuilder>
createAntBuilder(SScene &scene, std::shared_ptr<Renderer::IPxrMaterial> copper) {
  auto builder = scene.createArticulationBuilder();
  auto body = builder->createLinkBuilder();
  body->addSphereShape({{0, 0, 0}, PxIdentity}, 0.25);
  body->addSphereVisualWithMaterial({{0, 0, 0}, PxIdentity}, 0.25, copper);
  body->addCapsuleShape({{0.141, 0, 0}, PxIdentity}, 0.08, 0.141);
  body->addCapsuleVisualWithMaterial({{0.141, 0, 0}, PxIdentity}, 0.08, 0.141, copper);
  body->addCapsuleShape({{-0.141, 0, 0}, PxIdentity}, 0.08, 0.141);
  body->addCapsuleVisualWithMaterial({{-0.141, 0, 0}, PxIdentity}, 0.08, 0.141, copper);
  body->addCapsuleShape({{0, 0.141, 0}, physx::PxQuat(PI / 2, {0, 0, 1})}, 0.08, 0.141);
  body->addCapsuleVisualWithMaterial({{0, 0.141, 0}, physx::PxQuat(PI / 2, {0, 0, 1})}, 0.08,
                                     0.141, copper);
  body->addCapsuleShape({{0, -0.141, 0}, physx::PxQuat(PI / 2, {0, 0, 1})}, 0.08, 0.141);
  body->addCapsuleVisualWithMaterial({{0, -0.141, 0}, physx::PxQuat(PI / 2, {0, 0, 1})}, 0.08,
                                     0.141, copper);
  body->setName("body");

  auto l1 = builder->createLinkBuilder(body);
  l1->setName("l1");
  l1->setJointName("j1");
  l1->setJointProperties(PxArticulationJointType::eREVOLUTE, {{-0.5236, 0.5236}},
                         {{0.282, 0, 0}, {0, 0.7071068, 0, 0.7071068}},
                         {{0.141, 0, 0}, {0, -0.7071068, 0, 0.7071068}});
  l1->addCapsuleShape({{0, 0, 0}, PxIdentity}, 0.08, 0.141);
  l1->addCapsuleVisualWithMaterial({{0, 0, 0}, PxIdentity}, 0.08, 0.141, copper);

  auto l2 = builder->createLinkBuilder(body);
  l2->setName("l2");
  l2->setJointName("j2");
  l2->setJointProperties(PxArticulationJointType::eREVOLUTE, {{-0.5236, 0.5236}},
                         {{-0.282, 0, 0}, {0.7071068, 0, -0.7071068, 0}},
                         {{0.141, 0, 0}, {0, -0.7071068, 0, 0.7071068}});
  l2->addCapsuleShape({{0, 0, 0}, PxIdentity}, 0.08, 0.141);
  l2->addCapsuleVisualWithMaterial({{0, 0, 0}, PxIdentity}, 0.08, 0.141, copper);

  auto l3 = builder->createLinkBuilder(body);
  l3->setName("l3");
  l3->setJointName("j3");
  l3->setJointProperties(PxArticulationJointType::eREVOLUTE, {{-0.5236, 0.5236}},
                         {{0, 0.282, 0}, {-0.5, 0.5, 0.5, 0.5}},
                         {{0.141, 0, 0}, {0, -0.7071068, 0, 0.7071068}});
  l3->addCapsuleShape({{0, 0, 0}, PxIdentity}, 0.08, 0.141);
  l3->addCapsuleVisualWithMaterial({{0, 0, 0}, PxIdentity}, 0.08, 0.141, copper);

  auto l4 = builder->createLinkBuilder(body);
  l4->setName("l4");
  l4->setJointName("j4");
  l4->setJointProperties(PxArticulationJointType::eREVOLUTE, {{-0.5236, 0.5236}},
                         {{0, -0.282, 0}, {0.5, 0.5, -0.5, 0.5}},
                         {{0.141, 0, 0}, {0, -0.7071068, 0, 0.7071068}});
  l4->addCapsuleShape({{0, 0, 0}, PxIdentity}, 0.08, 0.141);
  l4->addCapsuleVisualWithMaterial({{0, 0, 0}, PxIdentity}, 0.08, 0.141, copper);

  auto f1 = builder->createLinkBuilder(l1);
  f1->setName("f1");
  f1->setJointName("j11");
  f1->setJointProperties(PxArticulationJointType::eREVOLUTE, {{0.5236, 1.222}},
                         {{-0.141, 0, 0}, {0.7071068, 0.7071068, 0, 0}},
                         {{0.282, 0, 0}, {0.7071068, 0.7071068, 0, 0}});
  f1->addCapsuleShape({{0, 0, 0}, PxIdentity}, 0.08, 0.282);
  f1->addCapsuleVisualWithMaterial({{0, 0, 0}, PxIdentity}, 0.08, 0.282, copper);

  auto f2 = builder->createLinkBuilder(l2);
  f2->setName("f2");
  f2->setJointName("j21");
  f2->setJointProperties(PxArticulationJointType::eREVOLUTE, {{0.5236, 1.222}},
                         {{-0.141, 0, 0}, {0.7071068, 0.7071068, 0, 0}},
                         {{0.282, 0, 0}, {0.7071068, 0.7071068, 0, 0}});
  f2->addCapsuleShape({{0, 0, 0}, PxIdentity}, 0.08, 0.282);
  f2->addCapsuleVisualWithMaterial({{0, 0, 0}, PxIdentity}, 0.08, 0.282, copper);

  auto f3 = builder->createLinkBuilder(l3);
  f3->setName("f3");
  f3->setJointName("j31");
  f3->setJointProperties(PxArticulationJointType::eREVOLUTE, {{0.5236, 1.222}},
                         {{-0.141, 0, 0}, {0.7071068, 0.7071068, 0, 0}},
                         {{0.282, 0, 0}, {0.7071068, 0.7071068, 0, 0}});
  f3->addCapsuleShape({{0, 0, 0}, PxIdentity}, 0.08, 0.282);
  f3->addCapsuleVisualWithMaterial({{0, 0, 0}, PxIdentity}, 0.08, 0.282, copper);

  auto f4 = builder->createLinkBuilder(l4);
  f4->setName("f4");
  f4->setJointName("j41");
  f4->setJointProperties(PxArticulationJointType::eREVOLUTE, {{0.5236, 1.222}},
                         {{-0.141, 0, 0}, {0.7071068, 0.7071068, 0, 0}},
                         {{0.282, 0, 0}, {0.7071068, 0.7071068, 0, 0}});
  f4->addCapsuleShape({{0, 0, 0}, PxIdentity}, 0.08, 0.282);
  f4->addCapsuleVisualWithMaterial({{0, 0, 0}, PxIdentity}, 0.08, 0.282, copper);

  return builder;
}

int main() {
  Renderer::SVulkan2Renderer::setLogLevel("info");
  auto sim = std::make_shared<Simulation>();
  auto renderer =
      std::make_shared<Renderer::SVulkan2Renderer>(false, 1000, 1000, 4, "", "back", false);
  sim->setRenderer(renderer);
  Renderer::SVulkan2Window window(renderer, 800, 600, "../vulkan_shader/ibl");

  SceneConfig config;
  // config.gravity = {0, 0, 0};

  auto scene = sim->createScene(config);
  window.setScene(static_cast<Renderer::SVulkan2Scene *>(scene->getRendererScene()));
  window.setCameraParameters(0.1, 100, 1);

  scene->addGround(0);
  scene->getRendererScene()->setAmbientLight({0.3, 0.3, 0.3});
  scene->getRendererScene()->addDirectionalLight({0, 1, -1}, {0.5, 0.5, 0.5}, true, {0, 0, 0}, 10,
                                                 -10, 10, 1024);

  auto builder = scene->createActorBuilder();
  builder->addBoxShape({{0, 0, 0}, {0.3305881, 0.1652941, 0.0991764, 0.9238795}});
  builder->addBoxVisual({{0, 0, 0}, {0.3305881, 0.1652941, 0.0991764, 0.9238795}});
  auto box = builder->build();
  box->setPose({{0, 0, 2}, PxIdentity});
  box->setName("box");

  scene->updateRender();
  scene->step();
  window.render("Color");
  window.mCameraController->setXYZ(-4, 0, 0.5);
  int iter = 0;
  while (1) {
    scene->updateRender();
    scene->step();

    for (auto c : scene->getContacts()) {
      std::cout << iter << " " << c->actors[0]->getName() << " " << c->actors[1]->getName()
                << std::endl;
    }

    if (iter==300) {
      scene->removeActor(box);
    }

    if (iter == 305) {
      return 0;
    }

    // window.render("Color");
    // if (window.windowCloseRequested()) {
    //   window.close();
    //   break;
    // }
    iter++;
  }

  return 0;
}
