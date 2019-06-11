#include <PxPhysicsAPI.h>
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

#include "optifuser_renderer.h"
#include "simulation.h"

using namespace physx;

int main(int argc, char **argv) {
  OptifuserRenderer renderer;
  renderer.init();

  PxSimulation sim;
  sim.setRenderer(&renderer);
  sim.setTimestep(1.f / 600.f);
  sim.addObj("/home/fx/mobility_mesh/resources/46437-4/objs/original-1.obj");

  while (true) {
    for (int i = 0; i < 10; ++i) {
      sim.step();
    }
    renderer.render();
  }
}
