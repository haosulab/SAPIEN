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

#define PI (3.141592653589793238462643383279502884f)

using namespace sapien;

int main() {
  Simulation sim;
  Renderer::SapienVulkanRenderer renderer;
  sim.setRenderer(&renderer);
  Renderer::SapienVulkanController controller(&renderer);

  // controller.showWindow();

  auto s0 = sim.createScene();
  s0->setTimestep(1 / 60.f);
  s0->addGround(-1);

  auto r0 = static_cast<Renderer::SapienVulkanScene *>(s0->getRendererScene());
  r0->setAmbientLight({0, 0, 0});
  r0->setShadowLight({0, -1, -1}, {1, 1, 1});

  {
    auto builder = s0->createActorBuilder();
    builder->addSphereShape();
    builder->addSphereVisual();
    builder->build();
  }

  auto builder = s0->createActorBuilder();
  builder->addVisualFromFile("/home/fx/Scenes/sponza/sponza.obj", PxTransform(), {0.001, 0.001, 0.001});
  builder->buildStatic();

  auto a = s0->createActorBuilder()->build(true);
  a->setPose(PxTransform({0, 0, 0}, PxIdentity));
  auto cam = s0->addMountedCamera("test", a, PxTransform(), 800, 600, 0.f, 1.f);

  // controller.setCameraPosition(-5, 0, 0);
  controller.setScene(s0.get());
  // controller.focus(ant0->getRootLink());

  while (!controller.isClosed()) {
    cam->takePicture();
    cam->getColorRGBA();

    s0->updateRender();
    s0->step();
    controller.render();
  }
  return 0;

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

}
