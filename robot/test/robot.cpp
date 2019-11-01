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

  renderer.cam.position = {0.5, -4, 0.5};
  renderer.cam.setForward({0, 1, 0});
  renderer.cam.setUp({0, 0, 1});
  renderer.setAmbientLight({.4, .4, .4});
  renderer.setShadowLight({1, -1, -1}, {.5, .5, .5});
  renderer.addPointLight({2, 2, 2}, {1, 1, 1});
  renderer.addPointLight({2, -2, 2}, {1, 1, 1});
  renderer.addPointLight({-2, 0, 2}, {1, 1, 1});

  Simulation sim;
  sim.setRenderer(&renderer);
  sim.setTimestep(1.f / 500.f);
  sim.addGround(0.0);

  auto builder = sim.createActorBuilder();
  auto loader = sim.createURDFLoader();
  loader->balancePassiveForce = true;
  std::string partFile =
      "/home/sim/project/mobility-v0-prealpha3/mobility_verified/44826/mobility.urdf";
  loader->fixLoadedObject = true;
  loader->load(partFile)->articulation->teleportRootLink({{3.0, 0.3, 0.8}, PxIdentity}, true);
  auto wrapper = loader->load("../assets/robot/single_hand.urdf");
    for(auto &actor: wrapper->get_links()){
      actor->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
    }
  wrapper->set_drive_property(200, 55);

  auto timestep = sim.getTimestep();
  auto uniqueWrapper = std::make_unique<ControllableArticulationWrapper>(wrapper);
  ControllableArticulationWrapper *controllableWrapper = uniqueWrapper.get();
  sim.mControllableArticulationWrapper.push_back(std::move(uniqueWrapper));
  controllableWrapper->updateTimeStep(timestep);

  //   ROS
  std::vector<std::string> serviceJoints = {
      "right_gripper_finger1_joint", "right_gripper_finger2_joint", "right_gripper_finger3_joint"};
  //  robot::ControllerManger manger("movo", controllableWrapper);
  //  manger.createJointPubNode(100, 500);
  //  auto IKController = manger.createCartesianVelocityController("right_arm");
  //  IKController->setAngularVelocity(0.2);
  //  IKController->setVelocity(0.2);
  //  auto jointController = manger.createJointVelocityController(serviceJoints, "right_gripper");
  //  manger.addGroupTrajectoryController("right_arm");
  //  manger.start();
  //  auto bodyController = manger.createJointVelocityController({"linear_joint"}, "body");
  //  auto planner = manger.createGroupPlanner("right_arm");

  wrapper->set_qpos({0.47, 0.47, 0.47, 1, 1, 1,0,0,0});
  wrapper->articulation->teleportRootLink({{0, 0, 1}, PxIdentity}, true);
  //  wrapper->set_drive_target({0.2, 0, 0.0, 0, -0.9, 0.1, 0.1, 0.1, 0, 0, 0, 0, 0});
  for (int j = 0; j < 100; ++j) {
    sim.step();
  }
  sim.step();

  //  wrapper->set_qf({10,10,10,10,10,10,10,10,10,10,10,10,10});

  sim.updateRenderer();
  renderer.showWindow();

  //  auto pose = wrapper->linkName2Link["right_ee_link"]->getGlobalPose();
  //  pose.p.x -= 0.2;
  //  planner->go(pose, "base_link");

  static size_t globalTimeStep = 0;
  while (true) {
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
