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

  auto s0 = sim.createScene();
  s0->setTimestep(1 / 480.f);
  s0->addGround(-1);

  // auto builder = createAntBuilder(*s0);
  auto loader = s0->createURDFLoader();
  loader->fixRootLink = 1;
  // auto a = loader->loadKinematic("../assets/robot/all_robot.urdf");
  loader->collisionIsVisual = 1;
  auto a = loader->load("../assets/robot/bullet_human.urdf");
  a->setRootPose({{0, 0, 2}, PxIdentity});

  // auto r0 = static_cast<Renderer::OptifuserScene *>(s0->getRendererScene());
  s0->setAmbientLight({0.3, 0.3, 0.3});
  s0->setShadowLight({0, -1, -1}, {.5, .5, 0.4});

  controller.setCameraPosition(-5, 0, 0);

  controller.setCurrentScene(s0.get());

  int count = 0;
  while (!controller.shouldQuit()) {

    for (int i = 0; i < 8; ++i) {
      s0->step();
    }
    s0->updateRender();
    controller.render();
  }

  return 0;
}
