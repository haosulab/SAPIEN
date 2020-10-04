#include "actor_builder.h"
#include "articulation_builder.h"
#include "controllable_articulation_wrapper.h"
#include "controller/cartesian_velocity_controller.h"
#include "controller/controller_manger.h"
#include "controller/velocity_control_service.h"
#include "device/joystick_ps3.h"
#include "optifuser_renderer.h"
#include "simulation.h"
#include <extensions/PxDefaultCpuDispatcher.h>
#include <extensions/PxSimpleFactory.h>
#include <optifuser.h>
#include <thread>
#include <vector>

using namespace sapien;

void test1() {
  Renderer::OptifuserRenderer renderer;
  
  renderer.cam.position = {0.5, -4, 0.5};
  renderer.cam.setForward({0, 1, 0});
  renderer.cam.setUp({0, 0, 1});

  Simulation sim;
  sim.setRenderer(&renderer);
  sim.setTimestep(1.f / 500.f);
  sim.addGround(0.0);

  PS3 input;
  while (true) {
    usleep(10000);
    sim.step();
    sim.updateRenderer();
    renderer.render();
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
