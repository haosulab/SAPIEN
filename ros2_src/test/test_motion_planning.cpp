#include "articulation/sapien_link.h"
#include "controller/sapien_controllable_articulation.h"
#include "manager/robot_descriptor.h"
#include "manager/robot_loader.h"
#include "manager/robot_manager.h"
#include "manager/scene_manager.h"
#include "renderer/optifuser_controller.h"
#include "renderer/optifuser_renderer.h"
#include "sapien_scene.h"
#include "simulation.h"
#include "spdlog/sinks/stdout_color_sinks.h"

using namespace sapien;
void test1(int argc, char *argv[]) {
  rcutils_logging_set_default_logger_level(30);
  auto logger = spdlog::stdout_color_mt("SAPIEN_ROS2");
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

  // ROS2 specified class
  rclcpp::init(argc, argv);
  ros2::SceneManager sceneManager(scene.get(), "scene1");
  ros2::RobotDescriptor descriptor("sapien_resources", "xarm6_description/urdf/xarm6.urdf",
                                   "xarm6_moveit_config/config/xarm6.srdf", true);

  auto loader = sceneManager.createRobotLoader();
  auto [robot, robotManager] = loader->loadRobotAndManager(descriptor, "xarm6");

  robot->setRootPose({{0, 0, 0.5}, PxIdentity});
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

  // Test motion planning
  ros2::MotionPlanningConfig config;
  ros2::setResourcesDirectory("/home/sim/project/sapien/python/py_ros2_package/ros2");
  config.max_acceleration_scaling_factor = 0.6;
  robotManager->setMotionPlanningConfig(config);
  ros2::MotionPlanner *planner = robotManager->buildMotionPlanner("arm", "NOT_USED");
  sceneManager.start();

  uint32_t step = 0;
  controller.showWindow();
  while (!controller.shouldQuit()) {
    scene->step();
    scene->updateRender();
    controller.render();
    step++;

    if (step == 1000) {
      planner->setStartStateToCurrentState();
      auto pose = robot->getSLinks()[7]->getPose();
      pose.p.x += 0.2;
      planner->setGoalState(pose);
      auto plan = planner->plan();
      std::cout << plan.duration << "\n\n"
//                << plan.jointNames << "\n\n"
                << plan.effort << "\n\n"
                << plan.acceleration << "\n\n"
                << plan.position << "\n\n"
                << plan.velocity;
    }

    // Balance force
    robotManager->balancePassiveForce();
    if (step >= 500 && step < 1000) {
      gripperController->moveJoint({5, 5, 5, 5, 5, 5});
    }
  }
  //  rclcpp::shutdown();
}

int main(int argc, char *argv[]) { test1(argc, argv); }
