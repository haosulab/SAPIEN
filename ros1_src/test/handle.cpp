#include "actor_builder.h"
#include "articulation/sapien_articulation.h"
#include "articulation/sapien_link.h"
#include "sapien_actor.h"
#include "sapien_drive.h"
#include "sapien_scene.h"
#include "simulation.h"

#include <optifuser_controller.h>
#include <optifuser_renderer.h>
#include <ros/callback_queue.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include "manager/robot_descriptor.h"
#include "manager/robot_loader.h"
#include "manager/robot_manager.h"
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

  s0->setAmbientLight({0.5, 0.5, 0.5});
  s0->setShadowLight({0, -1, -1}, {1, 1, 1});

  // controller.pause(true);
  controller.setCurrentScene(s0.get());
  controller.showWindow();

  // Scene Manager
  ros::init(argc, argv, "test_sapien_hande");
  ros1::SceneManager sceneManager(s0.get(), "sapien_sim", 4);
  auto loader = sceneManager.createRobotLoader();
  URDF::URDFConfig config;
  auto [robot, robotManager] = loader->loadFromParameterServer("ur5e", config, 125);
  robotManager->setDriveProperty(3000, 500, 1000, {0, 1, 2, 3, 4, 5});
  robotManager->setDriveProperty(50, 20, 100, {6, 7, 8, 9, 10, 11, 12, 13});
  std::vector<physx::PxReal> initQpos = {0, -1.57, 0, -1.57, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  robot->setQpos(initQpos);
  robot->setDriveTarget(initQpos);
//  s0->step();

  sceneManager.startAllCamera(30);
  sceneManager.start();
  while (!controller.shouldQuit()) {
    robotManager->balancePassiveForce();
    s0->updateRender();
    s0->step();
    controller.render();
  }

  return 0;
}
