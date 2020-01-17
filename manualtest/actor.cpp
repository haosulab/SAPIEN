#include "actor_builder.h"
#include "renderer/optifuser_controller.h"
#include "renderer/optifuser_renderer.h"
#include "sapien_actor.h"
#include "sapien_scene.h"
#include "simulation.h"

using namespace sapien;

int main() {
  Simulation sim;
  Renderer::OptifuserRenderer renderer;
  sim.setRenderer(&renderer);
  Renderer::OptifuserController controller(&renderer);

  controller.showWindow();

  auto s0 = sim.createScene("Scene 1");
  s0->addGround(-1);
  s0->setTimestep(1 / 60.f);

  auto s1 = sim.createScene("Scene 2");
  s1->addGround(-1);

  auto builder = s0->createActorBuilder();

  builder->addBoxShape();
  builder->addBoxVisual();

  auto actor = builder->build();
  actor->setPose({{0, 0, 2}, PxIdentity});

  auto r0 = static_cast<Renderer::OptifuserScene *>(s0->getRendererScene());
  r0->setAmbientLight({0.3, 0.3, 0.3});
  r0->setShadowLight({0, -1, -1}, {.5, .5, 0.4});
  controller.mCamera.position = {-5, 0, 0};

  controller.setCurrentScene(s0.get());

  while (!controller.shouldQuit()) {
    s0->updateRender();
    s0->step();

    controller.render();
  }

  return 0;
}
