#include <thread>
#include "actor_builder.h"
#include "articulation_builder.h"
#include "controller/controller_manger.h"
#include "device/movo_keyboard.h"
#include "optifuser_renderer.h"
#include "simulation.h"
#include <optifuser.h>
#include <thread>
using namespace sapien;
void run() {
  Renderer::OptifuserRenderer renderer;
  
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

  auto builder = sim.createActorBuilder();
  builder->addBoxShape({{0, 0, 0}, PxIdentity}, {0.5, 1.5, 0.3});
  builder->addBoxVisual({{0, 0, 0}, PxIdentity}, {0.5, 1.5, 0.3});
  auto actor = builder->build(false, false, "test", true);
  actor->setGlobalPose({{2.0, 0.3, 0.3}, PxIdentity});

  auto builder1 = sim.createActorBuilder();
  builder1->addObjVisual("../assets/object/029_plate/google_16k/textured.dae");
  builder1->addConvexShapeFromObj("../assets/object/029_plate/google_16k/textured.obj");
  auto plate = builder1->build(false, false, "plate", true);
  plate->setGlobalPose({{2.0, 0.3, 2}, PxIdentity});

  loader->load("../assets/46627/test.urdf")
      ->articulation->teleportRootLink({{2.0, 5.3, 0.4}, PxIdentity}, true);
  auto wrapper = loader->load("../assets/robot/all_robot.urdf");
  wrapper->set_drive_property(2000, 500);

  auto controllableWrapper = sim.createControllableArticulationWrapper(wrapper);
  auto manger = std::make_unique<robot::ControllerManger>("movo", controllableWrapper);
  robot::MOVOKeyboard keyboard(manger.get());

  renderer.showWindow();
  wrapper->set_qpos({0.47, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
  wrapper->set_drive_target({0.2, 0, 0.0, 0, -0.9, 0.1, 0.1, 0.1, 0, 0, 0, 0, 0});

  while (true) {
    sim.step();
    sim.updateRenderer();
    renderer.render();
    keyboard.step();

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
