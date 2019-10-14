#include "actor_builder.h"
#include "articulation_builder.h"
#include "controller/controller_manger.h"
#include "device/movo_ps3.h"
#include "optifuser_renderer.h"
#include "simulation.h"
#include <extensions/PxDefaultCpuDispatcher.h>
#include <extensions/PxSimpleFactory.h>
#include <optifuser.h>
#include <thread>
#include <vector>

using namespace sapien;
void run() {
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
  auto wrapper = loader->load("../assets/robot/all_robot.urdf");
  wrapper->set_drive_property(2000, 500);

  auto controllableWrapper = sim.createControllableArticulationWrapper(wrapper);
  auto manger = std::make_unique<robot::ControllerManger>("movo", controllableWrapper);

  robot::MOVOPS3 ps3(manger.get());

  renderer.showWindow();
  wrapper->set_qpos({0.47, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
  wrapper->set_drive_target({0.2, 0, 0.0, 0, -0.9, 0.1, 0.1, 0.1, 0, 0, 0, 0, 0});
  sim.step();

  while (true) {
    sim.step();
    sim.updateRenderer();
    renderer.render();
    ps3.step();

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
