#include "articulation/articulation_builder.h"
#include "articulation/sapien_articulation.h"
#include "articulation/sapien_link.h"
#include "renderer/svulkan2_renderer.h"
#include "renderer/svulkan2_window.h"
#include "sapien_actor.h"
#include "sapien_drive.h"
#include "sapien_scene.h"
#include "simulation.h"
#include <iostream>

#define PI (3.141592653589793238462643383279502884f)

using namespace sapien;

std::unique_ptr<ArticulationBuilder>
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
  auto sim = std::make_shared<Simulation>();
  auto renderer = std::make_shared<Renderer::SVulkan2Renderer>(false, 1000, 1000, 4);
  sim->setRenderer(renderer);
  Renderer::SVulkan2Window window(renderer, 800, 600, "../vulkan_shader/full");

  auto s0 = sim->createScene();
  s0->setTimestep(1 / 60.f);
  s0->addGround(-1);

  auto copper = renderer->createMaterial();
  copper->setBaseColor({0.975, 0.453, 0.221, 1});
  copper->setMetallic(1.f);
  copper->setRoughness(0.7f);
  copper->setSpecular(0.5f);

  auto builder = createAntBuilder(*s0, copper);

  auto ant0 = builder->build(false);
  ant0->setName("ant0");
  ant0->setRootPose({{0, 0, 2}, PxIdentity});

  auto r0 = static_cast<Renderer::SVulkan2Scene *>(s0->getRendererScene());
  auto enableShadow = true;

  // r0->setAmbientLight({0.5, 0.5, 0.5});
  r0->addDirectionalLight({0, -1, -1}, {1, 1, 1}, enableShadow, {0, 0, 0}, 20.f, -20.f, 20.f);

  r0->addPointLight({1, 1, 2}, {10, 0, 0}, enableShadow, 0.1, 20);
  r0->addPointLight({-1, 1, 2}, {0, 10, 0}, enableShadow, 0.1, 20);
  r0->addPointLight({1, -1, 2}, {0, 0, 10}, enableShadow, 0.1, 20);

  window.setScene(r0);

  int count = 0;
  SDrive *drive;
  std::vector<float> d;
  while (1) {
    if (++count == 120) {
      d = ant0->packData();
      drive = s0->createDrive(nullptr, {{0, 0, 0}, PxIdentity}, ant0->getRootLink(),
                              {{0, 0, 0}, PxIdentity});
      drive->setProperties(2000, 4000, PX_MAX_F32, true);
      drive->setTarget({{0, 0, 1}, PxQuat(1, {0, 1, 0})});
    }

    s0->updateRender();
    s0->step();
    window.render("Color");
    if (window.windowCloseRequested()) {
      window.close();
      break;
    }

    if (count == 180) {
      spdlog::get("SAPIEN")->info("Trying to resize at step 180.");
      window.resize(300, 600);
    }

  }

  return 0;
}
