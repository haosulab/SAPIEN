#include "articulation/sapien_articulation.h"
#include "articulation/urdf_loader.h"
#include "controller/sapien_controllable_articulation.h"
#include "controller/scene_manager.h"
#include "renderer/optifuser_controller.h"
#include "renderer/optifuser_renderer.h"
#include "sapien_scene.h"
#include "scene.h"
#include "simulation.h"

using namespace sapien;
void test1(int argc, char *argv[]) {
  Simulation sim;
  Renderer::OptifuserRenderer renderer;
  sim.setRenderer(&renderer);

  auto controller = Renderer::OptifuserController(&renderer);
  controller.showWindow();

  auto scene = sim.createScene();
  scene->setShadowLight({0, 1, -1}, {0.5, 0.5, 0.5});
  scene->setAmbientLight({0.5, 0.5, 0.5});
  controller.setCameraPosition(-0.5, 2, 0.5);
  controller.setCameraRotation(-1, 0);
  controller.setCurrentScene(scene.get());
  scene->addGround(0);
  scene->setTimestep(1.0 / 60);

  auto loader = scene->createURDFLoader();
  loader->fixRootLink = true;
  loader->scale = 1.0;
  auto robot = loader->load("../assets/robot/xarm6.urdf");
  robot->setRootPose({{0, 0, 0.5}, PxIdentity});

  // ROS2 specified class
  rclcpp::init(argc, argv);
  ros2::SceneManager sceneManager(scene.get(), "scene1");
  auto robotManager = sceneManager.buildRobotManager(robot, "movo");
  robotManager->setDriveProperty(0, 50);
  auto controllableWrapper = robotManager->wrapper;

  // Test Controller
  robotManager->createJointPublisher(20);
  sceneManager.start();

  uint32_t step = 0;
  while (!controller.shouldQuit()) {
    scene->step();
    scene->updateRender();
    controller.render();
    step++;
    robotManager->balancePassiveForce();
//    std::cout << sceneManager.now().nanoseconds() / 1e9 << std::endl;
    if (step == 100) {
      std::vector<float> temp(robot->dof(), -1);
      controllableWrapper->mPositionCommands.push(temp);
    }
  }
  rclcpp::shutdown();
}

int main(int argc, char *argv[]) { test1(argc, argv); }