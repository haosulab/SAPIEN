//
// Created by sim on 9/27/19.
//

#include "actor_builder.h"
#include "articulation_builder.h"
#include "controllable_articulation_wrapper.h"
#include "controller/cartesian_velocity_controller.h"
#include "controller/controller_manger.h"
#include "controller/velocity_control_service.h"
#include "optifuser_renderer.h"
#include "simulation.h"
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
  sim.setTimestep(1.f / 500.f);
  sim.addGround(0.0);

  auto builder = sim.createActorBuilder();
  //  builder->addBoxShape({{0, 0, 0}, PxIdentity}, {0.3, 1.5, 0.1});
  //  builder->addBoxVisual({{0, 0, 0}, PxIdentity}, {0.3, 1.5, 0.1});
  //  auto actor = builder->build(false, false, "test", true);
  //  actor->setGlobalPose({{1.0, 0.3, 1}, PxIdentity});

  PS3::PS3Input input;
  auto loader = sim.createURDFLoader();
  loader->balancePassiveForce = true;
  // loader->load("../assets/46627/test.urdf")
  //     ->articulation->teleportRootLink({{1.0, 0.3, 0.4}, PxIdentity}, true);
  auto wrapper = loader->load("../assets/robot/all_robot.urdf");
  wrapper->set_drive_property(2000, 500);

  auto timestep = sim.getTimestep();
  auto uniqueWrapper = std::make_unique<ControllableArticulationWrapper>(wrapper);
  ControllableArticulationWrapper *controllableWrapper = uniqueWrapper.get();
  sim.mControllableArticulationWrapper.push_back(std::move(uniqueWrapper));
  controllableWrapper->updateTimeStep(timestep);

  //   ROS
  std::vector<std::string> serviceJoints = {
      "right_gripper_finger1_joint", "right_gripper_finger2_joint", "right_gripper_finger3_joint"};
  robot::ControllerManger manger("movo", controllableWrapper);
  manger.createJointPubNode(100, 500);
  auto IKController = manger.createCartesianVelocityController("right_arm");
  IKController->setAngularVelocity(0.2);
  IKController->setVelocity(0.2);
  auto jointController = manger.createJointVelocityController(serviceJoints, "right_gripper");
  manger.createGroupTrajectoryController("right_arm");
  manger.start();
  auto bodyController = manger.createJointVelocityController({"linear_joint"}, "body");

  wrapper->set_qpos({0.47, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
  wrapper->set_drive_target({0.2, 0, 0.0, 0, -0.9, 0.1, 0.1, 0.1, 0, 0, 0, 0, 0});
  sim.step();

  //  wrapper->set_qf({10,10,10,10,10,10,10,10,10,10,10,10,10});

  sim.updateRenderer();
  bool continuous = true;
  float gripperVelocity = 5;
  float bodyVelocity = 0.1;
  renderer.showWindow();

  static size_t globalTimeStep = 0;
  while (true) {
//    if (input.getKey(PS3::BUTTON_UP)) {
//      IKController->moveRelative(robot::CartesianCommand::X_F, continuous);
//    } else if (input.getKey(PS3::BUTTON_DOWN)) {
//      IKController->moveRelative(robot::CartesianCommand::X_B, continuous);
//    } else if (input.getKey(PS3::BUTTON_LEFT)) {
//      IKController->moveRelative(robot::CartesianCommand::Y_F, continuous);
//    } else if (input.getKey(PS3::BUTTON_RIGHT)) {
//      IKController->moveRelative(robot::CartesianCommand::Y_B, continuous);
//    } else if (input.getKey(PS3::BUTTON_L1)) {
//      IKController->moveRelative(robot::CartesianCommand::Z_F, continuous);
//    } else if (input.getKey(PS3::BUTTON_L2)) {
//      IKController->moveRelative(robot::CartesianCommand::Z_B, continuous);
//    } else if (input.getKey(PS3::BUTTON_CIRCLE)) {
//      jointController->moveJoint(serviceJoints, gripperVelocity);
//    } else if (input.getKey(PS3::BUTTON_SQUARE)) {
//      jointController->moveJoint(serviceJoints, -gripperVelocity);
//    } else if (input.getKey(PS3::BUTTON_R1)) {
//      IKController->moveRelative(robot::CartesianCommand::ROLL_F, continuous);
//    } else if (input.getKey(PS3::BUTTON_R2)) {
//      IKController->moveRelative(robot::CartesianCommand::ROLL_B, continuous);
//    } else if (input.getKey(PS3::BUTTON_X)) {
//      bodyController->moveJoint({"linear_joint"}, -bodyVelocity);
//    } else if (input.getKey(PS3::BUTTON_TRIANGLE)) {
//      bodyController->moveJoint({"linear_joint"}, bodyVelocity);
//    }
    sim.step();
    globalTimeStep++;
    sim.updateRenderer();
    usleep(1000);
    if (globalTimeStep % 3 == 0) {
      renderer.render();
    }
    auto gl_input = Optifuser::getInput();
    if (gl_input.getKeyState(GLFW_KEY_Q)) {
      break;
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot");
  test1();
}
