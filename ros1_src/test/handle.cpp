#include "actor_builder.h"
#include "articulation/sapien_articulation.h"
#include "articulation/sapien_link.h"
#include "sapien_actor.h"
#include "sapien_drive.h"
#include "sapien_scene.h"
#include "simulation.h"
#include <optifuser_controller.h>
#include <optifuser_renderer.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include "manager/robot_descriptor.h"
#include "manager/scene_manager.h"

using namespace sapien;
int main(int argc, char **argv) {
  auto logger = spdlog::stdout_color_mt("SAPIEN_ROS1");
  Simulation sim;
  Renderer::OptifuserRenderer renderer;
  sim.setRenderer(&renderer);
  Renderer::OptifuserController controller(&renderer);

  auto s0 = sim.createScene();
  s0->setTimestep(1 / 250.f);
  s0->addGround(0);

  s0->setAmbientLight({0, 0, 0});
  s0->setShadowLight({0, -1, -1}, {1, 1, 1});

  auto a = s0->createActorBuilder()->build(true);
  a->setPose(PxTransform({-2, 0, 0}, PxIdentity));
  s0->addMountedCamera("test", a, PxTransform(), 800, 600, 0.f, 1.f);
  controller.setCurrentScene(s0.get());

  // controller.pause(true);
  controller.showWindow();

  // Scene Manager
  ros::init(argc, argv, "test_sapien_hande");
  ros1::SceneManager sceneManager(s0.get(), "sapien_sim", 1);
  auto loader = sceneManager.createRobotLoader();
  URDF::URDFConfig config;
  auto [robot, robotManager] = loader->loadFromParameterServer("ur5e", config, 125);

  int count = 0;
  while (!controller.shouldQuit()) {
    s0->updateRender();
    s0->step();
    controller.render();
  }

  return 0;
}
