#include "articulation/articulation_builder.h"
#include "articulation/sapien_articulation.h"
#include "articulation/sapien_link.h"
#include "renderer/sapien_vulkan_controller.h"
#include "renderer/sapien_vulkan_renderer.h"
#include "sapien_actor.h"
#include "sapien_drive.h"
#include "sapien_scene.h"
#include "simulation.h"
#include <iostream>
#include "articulation/urdf_loader.h"

#define PI (3.141592653589793238462643383279502884f)

using namespace sapien;

std::unique_ptr<ArticulationBuilder> createAntBuilder(SScene &scene) {
  Renderer::PxrMaterial copper = {};
  copper.base_color = {0.975, 0.453, 0.221, 1};
  copper.metallic = 1.f;
  copper.roughness = 0.7f;
  copper.specular = 0.5f;

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
  Simulation sim;
  Renderer::SapienVulkanRenderer renderer;
  sim.setRenderer(&renderer);
  Renderer::SapienVulkanController controller(&renderer);

  // controller.showWindow();

  auto s0 = sim.createScene();
  s0->setTimestep(1 / 60.f);
  s0->addGround(-1);

  auto builder = createAntBuilder(*s0);

  auto ant0 = builder->build(false);
  ant0->setName("ant0");
  ant0->setRootPose({{0, 0, 2}, PxIdentity});

  auto r0 = static_cast<Renderer::SapienVulkanScene *>(s0->getRendererScene());
  r0->setAmbientLight({0.2, 0.2, 0.2});
  r0->setShadowLight({0, -1, -1}, {1, 1, 1});

  auto builder2 = s0->createActorBuilder();
  builder2->addSphereShape();
  builder2->addSphereVisual();
  auto ball = builder2->build(false, "ball");

  auto a = s0->createActorBuilder()->build(true);
  a->setPose(PxTransform({-2, 0, 0}, PxIdentity));
  s0->addMountedCamera("test", a, PxTransform(), 800, 600, 0.f, 1.f);

  controller.setScene(s0.get());
  controller.setFreeCameraPosition(-3, 0, 0);
  // controller.pause(true);

  auto loader = s0->createURDFLoader();
  loader->loadKinematic("../assets/shadow/kinova_shadow.urdf");

  int count = 0;
  SDrive *drive;
  std::vector<float> d;
  while (!controller.isClosed()) {

    if (++count == 120) {
      d = ant0->packData();
      drive = s0->createDrive(nullptr, {{0, 0, 0}, PxIdentity}, ant0->getRootLink(),
                              {{0, 0, 0}, PxIdentity});
      drive->setProperties(2000, 4000, PX_MAX_F32, true);
      drive->setTarget({{0, 0, 1}, PxQuat(1, {0, 1, 0})});
    }

    s0->updateRender();
    s0->step();
    controller.render();
  }

  return 0;
}

// int main() {
//   Simulation sim;
//   Renderer::SapienVulkanRenderer renderer;
//   sim.setRenderer(&renderer);
//   Renderer::SapienVulkanController controller(&renderer);

//   auto s0 = sim.createScene();
//   s0->addGround(-1);
//   s0->setTimestep(1 / 60.f);

//   auto s1 = sim.createScene();
//   s1->addGround(-1);

//   auto builder = s0->createActorBuilder();

//   builder->addMultipleConvexShapesFromFile("/home/fx/source/py-vhacd/example/decomposed.obj");
//   builder->addVisualFromFile("/home/fx/source/py-vhacd/example/decomposed.obj");
//   // builder->addDecomposedConvexShapesFromFile("../assets/shapenet/cup.dae",
//   PxTransform(PxIdentity),
//   //                                            {0.1, 0.1, 0.1});
//   // builder->addVisualFromFile("../assets/shapenet/cup.dae", PxTransform(PxIdentity),
//   //                            {0.1, 0.1, 0.1});

//   auto actor = builder->build();
//   actor->setPose({{0, 0, 2}, PxIdentity});

//   auto r0 = static_cast<Renderer::SapienVulkanScene *>(s0->getRendererScene());
//   r0->setAmbientLight({0.3, 0.3, 0.3});
//   r0->setShadowLight({0, -1, -1}, {.5, .5, 0.4});

//   controller.setScene(s0.get());

//   while (!controller.isClosed()) {
//     s0->updateRender();
//     s0->step();

//     controller.render();
//   }

//   return 0;
// }
