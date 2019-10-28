//
// Created by sim on 10/27/19.
//
#include "actor_builder.h"
#include "articulation_builder.h"
#include "optifuser_renderer.h"
#include "simulation.h"
#include "urdf/urdf_loader.h"
#include <PxPhysicsAPI.h>
#include <experimental/filesystem>
#include <extensions/PxDefaultAllocator.h>
#include <extensions/PxDefaultCpuDispatcher.h>
#include <extensions/PxDefaultErrorCallback.h>
#include <extensions/PxDefaultSimulationFilterShader.h>
#include <extensions/PxExtensionsAPI.h>
#include <extensions/PxShapeExt.h>
#include <extensions/PxSimpleFactory.h>
#include <foundation/PxMat33.h>
#include <iostream>
#include <object.h>
#include <optifuser.h>
#include <vector>

#include <random>

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

  auto articulationWrapper = loader.load("../assets/xarm/xarm6.urdf");


  printf("Simulation start\n");
  while (true) {
    sim.step();
    sim.updateRenderer();
    renderer.render();
    renderer.showWindow();

    if (Optifuser::getInput().getKeyState(GLFW_KEY_Q)) {
      break;
    }
  }
}

int main() {
  test3();
  return 0;
}