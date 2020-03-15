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
  auto robot = loader->load("../example/assets/robot/xarm6.urdf");
  robot->setRootPose({{0, 0, 0.5}, PxIdentity});

  // ROS2 specified class
  rclcpp::init(argc, argv);
  ros2::SceneManager sceneManager(scene.get(), "scene1");
  auto robotManager = sceneManager.buildRobotManager(robot, "xarm6");
  robotManager->setDriveProperty(1000, 50, 5000, {0, 1, 2, 3, 4, 5});
  robotManager->setDriveProperty(0, 50, 50, {6, 7, 8, 9, 10, 11});

  // Test Basic Controller
  std::vector<std::string> gripperJoints = {"drive_joint",
                                            "left_finger_joint",
                                            "left_inner_knuckle_joint",
                                            "right_outer_knuckle_joint",
                                            "right_finger_joint",
                                            "right_inner_knuckle_joint"};
  robotManager->createJointPublisher(20.0f);
  auto gripperController =
      robotManager->buildJointVelocityController(gripperJoints, "gripper_joint_velocity", 0.0f);
  sceneManager.start();

  // Test IK Controller
  auto armController =
      robotManager->buildCartesianVelocityController("arm", "arm_cartesian_velocity", 40.0f);

  // test PS3
  sceneManager.createPS3Publisher(50.0f);

  uint32_t step = 0;
  controller.showWindow();
  while (!controller.shouldQuit()) {
    scene->step();
    scene->updateRender();
    controller.render();
    step++;
    std::cout << step << std::endl;

    // Balance force
    robotManager->balancePassiveForce();
    // Move arm IK
    armController.lock()->moveCartesian({0.0, 0.02, 0.02}, ros2::MoveType::WorldTranslate);

    if (step >= 500 && step < 1000) {
      gripperController.lock()->moveJoint({5, 5, 5, 5, 5, 5});
    }
    if (step == 1000) {
      gripperController.lock()->moveJoint({-0.1, -0.1, -0.1, -0.1, -0.1, -0.1}, true);
    }
  }
  rclcpp::shutdown();
}

int main(int argc, char *argv[]) { test1(argc, argv); }