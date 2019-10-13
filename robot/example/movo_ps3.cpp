#include "actor_builder.h"
#include "articulation_builder.h"
#include "controller_manger.h"
#include "input/ps3.hpp"
#include "optifuser_renderer.h"
#include "simulation.h"
#include <extensions/PxDefaultCpuDispatcher.h>
#include <extensions/PxSimpleFactory.h>
#include <input/ps3.hpp>
#include <optifuser.h>
#include <thread>
#include <vector>

using namespace sapien;
void run() {
  PS3::PS3Input ps3;

  Renderer::OptifuserRenderer renderer;
  renderer.init();
  renderer.cam.position = {0.5, -4, 0.5};
  renderer.cam.setForward({0, 1, 0});
  renderer.cam.setUp({0, 0, 1});

  Simulation sim;
  sim.setRenderer(&renderer);
  sim.setTimestep(1.f / 300.f);
  sim.addGround(0.0);

  auto loader = sim.createURDFLoader();
  loader->fixLoadedObject = true;
  loader->balancePassiveForce = true;
  auto wrapper = loader->load("../../assets/robot/all_robot.urdf");

  auto controllableWrapper = sim.createControllableArticulationWrapper(wrapper);
  robot::ControllerManger manger("movo", controllableWrapper);
  manger.createJointPubNode(60, 500);
  manger.createGroupTrajectoryController("right_arm");
}
