//
// Created by sim on 9/27/19.
//

#include "actor_builder.h"
#include "articulation_builder.h"
#include "cartesian_velocity_controller.h"
#include "controllable_articulation_wrapper.h"
#include "controller_manger.h"
#include "joint_pub_node.h"
#include "joint_trajectory_controller.h"
#include "optifuser_renderer.h"
#include "simulation.h"
#include "urdf/urdf_loader.h"
#include "velocity_control_service.h"
#include <extensions/PxDefaultCpuDispatcher.h>
#include <extensions/PxSimpleFactory.h>
#include <input/ps3.hpp>
#include <optifuser.h>
#include <thread>
#include <vector>

using namespace sapien;

void test1() {
  Renderer::OptifuserRenderer renderer;
  renderer.init();
  renderer.cam.position = {0.5, -4, 0.5};
  renderer.cam.setForward({0, 1, 0});
  renderer.cam.setUp({0, 0, 1});

  Simulation sim;
  sim.setRenderer(&renderer);
  sim.setTimestep(1.f / 60.f);

  auto builder = sim.createActorBuilder();
  PxReal boxLength = 0.5f;
  PxTransform boxPose({0.5f + boxLength, 0.f, -0.5f + boxLength + 1.f}, PxIdentity);
  sim.addGround(0.0);

  PS3::PS3Input input;

  auto loader = URDF::URDFLoader(sim);
  //  loader.load("../assets/46627/test.urdf")
  //      ->articulation->teleportRootLink({{1.2, 0.2, 0.4}, PxIdentity}, true);

  auto wrapper = loader.loadKinematic("../assets/robot/all_robot.urdf");
  auto timestep = sim.getTimestep();
  auto uniqueWrapper = std::make_unique<ControllableArticulationWrapper>(wrapper);
  ControllableArticulationWrapper *controllableWrapper = uniqueWrapper.get();
  sim.mControllableArticulationWrapper.push_back(std::move(uniqueWrapper));
  controllableWrapper->updateTimeStep(timestep);

  // ROS
  std::vector<std::string> serviceJoints = {
      "right_gripper_finger1_joint", "right_gripper_finger2_joint", "right_gripper_finger3_joint"};
  robot::ControllerManger manger("movo", controllableWrapper);
  manger.createJointPubNode(100, 500);
//  manger.createCartesianVelocityController("right_arm");
//  manger.createJointVelocityController(serviceJoints, "right_gripper");
  manger.createGroupTrajectoryController("right_arm");
  manger.start();

  //  auto queue = controller.getQueue();
  //  std::vector<PxReal> initQpos = {-1.93475823254,
  //                                  -1.53188487338,
  //                                  0.951243371548,
  //                                  -2.24179359416,
  //                                  0.344180286477,
  //                                  0.649430580507,
  //                                  -1.41300076449,
  //                                  0,
  //                                  0,
  //                                  0};
  std::vector<PxReal> initQpos(13, 0);
  wrapper->set_qpos(initQpos);
  //    wrapper->set_drive_target(initQpos);
  //  wrapper->set_qvel({0.1, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0, 0, 0});
  //  wrapper->set_qvel({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  sim.step();
  sim.updateRenderer();
  sim.step();
  sim.updateRenderer();
  auto past = ros::Time::now();
  bool continuous = false;

  while (true) {
    //    if (input.getKey(PS3::BUTTON_UP)) {
    //      IKController.moveRelative(robot::CartesianCommand::X_F, continuous);
    //    } else if (input.getKey(PS3::BUTTON_DOWN)) {
    //      IKController.moveRelative(robot::CartesianCommand::X_B, continuous);
    //    } else if (input.getKey(PS3::BUTTON_LEFT)) {
    //      IKController.moveRelative(robot::CartesianCommand::Y_F, continuous);
    //    } else if (input.getKey(PS3::BUTTON_RIGHT)) {
    //      IKController.moveRelative(robot::CartesianCommand::Y_B, continuous);
    //    } else if (input.getKey(PS3::BUTTON_L1)) {
    //      IKController.moveRelative(robot::CartesianCommand::Z_F, continuous);
    //    } else if (input.getKey(PS3::BUTTON_L2)) {
    //      IKController.moveRelative(robot::CartesianCommand::Z_B, continuous);
    //    } else if (input.getKey(PS3::BUTTON_CIRCLE)) {
    //      IKController.toggleJumpTest(true);
    //    } else if (input.getKey(PS3::BUTTON_SQUARE)) {
    //      IKController.toggleJumpTest(false);
    //    }
    sim.step();
    sim.updateRenderer();
    usleep(1000);
    auto current = ros::Time::now();
//    std::cout << (current - past).toSec() << std::endl;
    past = current;
    renderer.render();
    auto gl_input = Optifuser::getInput();
    if (gl_input.getKeyState(GLFW_KEY_Q)) {
      break;
    }
    if (input.getKey(PS3::BUTTON_X)) {
      break;
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot");
  test1();
}