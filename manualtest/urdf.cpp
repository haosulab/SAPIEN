#include "articulation/articulation_builder.cpp"
#include "articulation/urdf_loader.h"
#include "renderer/optifuser_controller.h"
#include "renderer/optifuser_renderer.h"
#include "sapien_scene.h"
#include "simulation.h"
#include <iostream>

using namespace sapien;

int main() {
  Simulation sim;
  Renderer::OptifuserRenderer renderer;
  sim.setRenderer(&renderer);
  Renderer::OptifuserController controller(&renderer);

  controller.showWindow();

  auto s0 = sim.createScene("Scene 1");
  s0->setTimestep(1 / 60.f);
  s0->addGround(-1);

  // auto builder = createAntBuilder(*s0);
  auto loader = s0->createURDFLoader();
  auto a = loader->load("../assets/179/mobility.urdf");

  // auto s1 = builder->build(false);
  // s1->setName("Ant");
  // s1->setRootPose({{0, 0, 2}, PxIdentity});

  auto r0 = static_cast<Renderer::OptifuserScene *>(s0->getRendererScene());
  r0->setAmbientLight({0.3, 0.3, 0.3});
  r0->setShadowLight({0, -1, -1}, {.5, .5, 0.4});

  controller.mCamera.position = {-5, 0, 0};

  controller.setCurrentScene(s0.get());

  while (1) {
    s0->updateRender();
    s0->step();
    controller.render();
  }

  return 0;
}
