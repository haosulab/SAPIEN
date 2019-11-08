#include "actor_builder.h"
#include "articulation_builder.h"
#include "controller/controller_manger.h"
#include "device/xarm6.hpp"
#include "optifuser_renderer.h"
#include "simulation.h"
#include <optifuser.h>
#include <thread>

using namespace sapien;

void run() {
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
  sim.setTimestep(1.f / 300.f);
  sim.addGround(0.0);
  auto loader = sim.createURDFLoader();
  loader->fixLoadedObject = true;
  loader->balancePassiveForce = true;

  auto builder = sim.createActorBuilder();
  builder->addBoxShape({{0, 0, 0}, PxIdentity}, {0.5, 1.5, 0.3});
  builder->addBoxVisual({{0, 0, 0}, PxIdentity}, {0.5, 1.5, 0.3});
  auto actor = builder->build(false, false, "test", true);
  actor->setGlobalPose({{2.0, 0.3, 0.3}, PxIdentity});

  std::vector<std::string> gripperJoints = {"drive_joint",
                                            "left_finger_joint",
                                            "left_inner_knuckle_joint",
                                            "right_outer_knuckle_joint",
                                            "right_finger_joint",
                                            "right_inner_knuckle_joint"};

  auto wrapper = loader->load("../assets/robot/xarm6.urdf");
  wrapper->set_drive_property(1000, 300, 300, {0, 1, 2, 3, 4, 5});
  wrapper->set_drive_property(500, 100, 20, {6, 7, 8, 9, 10, 11});

  auto controllableWrapper = sim.createControllableArticulationWrapper(wrapper);
  auto manger = std::make_unique<robot::ControllerManger>("xarm6", controllableWrapper);
  manger->createJointPubNode(100, 1000);
  manger->createJointVelocityController(gripperJoints, "gripper");
  manger->createCartesianVelocityController("xarm6");
  manger->start();
  auto ps3 = robot::XArm6PS3(manger.get());

  wrapper->set_qpos({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  wrapper->set_drive_target({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});

  renderer.showWindow();
  std::vector<std::vector<PxReal>> temp;
  int i = 0;
  while (true) {
    sim.step();
    sim.updateRenderer();
    renderer.render();
    ps3.step();
    i++;
    if (i > 5000) {
      sim.clearCache();
      std::cout << "clear" << std::endl;
    }
    //    temp.push_back(sim.dump());

    auto gl_input = Optifuser::getInput();
    if (gl_input.getKeyState(GLFW_KEY_Q)) {
      break;
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ps3_movo");
  run();
  return 0;
}
