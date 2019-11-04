//
// Created by sim on 10/27/19.
//
#include "actor_builder.h"
#include "articulation_builder.h"
#include "controller/controller_manger.h"
#include "optifuser_renderer.h"
#include "simulation.h"
#include "urdf/urdf_loader.h"
#include <extensions/PxDefaultAllocator.h>
#include <optifuser.h>
#include <thread>

using namespace sapien;
void test3() {
  Renderer::OptifuserRenderer renderer;

  renderer.cam.position = {0, -2, 0.5};
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

  auto loader = URDF::URDFLoader(sim);
  loader.fixLoadedObject = true;
  renderer.showWindow();

  auto xarm = loader.load("../assets/robot/xarm6.urdf");
  xarm->set_drive_property(2000, 500);
  sim.addGround(-0.5);
  auto controllableWrapper = sim.createControllableArticulationWrapper(xarm);
  auto manger = std::make_unique<robot::ControllerManger>("xarm", controllableWrapper);
  manger->createJointPubNode(60, 500);
  std::vector<std::string> gripperNames = {"drive_joint",
                                           "left_finger_joint",
                                           "left_inner_knuckle_joint",
                                           "right_outer_knuckle_joint",
                                           "right_finger_joint",
                                           "right_inner_knuckle_joint"};
  auto gripper = manger->createJointVelocityController(gripperNames, "gripper");

  printf("Simulation start\n");
  while (true) {
    sim.step();
    sim.updateRenderer();
    renderer.render();
    gripper->moveJoint(gripperNames, 0.1);

        if (Optifuser::getInput().getKeyState(GLFW_KEY_Q)) {
      break;
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "xarm");
  test3();
  return 0;
}
