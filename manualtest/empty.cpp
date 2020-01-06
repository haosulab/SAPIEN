#include "actor_builder.h"
#include "renderer/optifuser_renderer.h"
#include "sapien_actor.h"
#include "sapien_scene.h"
#include "simulation.h"

using namespace sapien;

int main() {
  Simulation sim;
  Renderer::OptifuserRenderer renderer;
  sim.setRenderer(&renderer);

  renderer.showWindow();

  auto s0 = sim.createScene();
  s0->addGround(-1);

  auto builder = s0->createActorBuilder();

  builder->addBoxShape();
  builder->addBoxVisual();

  auto actor = builder->build();

  actor->getPxActor()->setGlobalPose({{0, 0, 5}, PxIdentity});

  auto r0 = static_cast<Renderer::OptifuserScene *>(s0->getRendererScene());
  r0->setAmbientLight({0.3, 0.3, 0.3});
  r0->setShadowLight({0, -1, -1}, {1, 1, 0.9});
  renderer.cam.position = {-5, 0, 0};

  while (1) {
    s0->updateRender();
    s0->step();
    renderer.render(*static_cast<Renderer::OptifuserScene *>(s0->getRendererScene()));
  }

  return 0;
}
