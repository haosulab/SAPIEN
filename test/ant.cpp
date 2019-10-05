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
using namespace physx;

int main() {
  Renderer::OptifuserRenderer renderer;
  renderer.init();

  renderer.cam.position = {0, -5, 1};
  renderer.cam.setForward({0, 1, 0});
  renderer.cam.setUp({0, 0, 1});
  renderer.cam.rotateYawPitch(0, -0.3);

  Simulation sim;
  sim.setRenderer(&renderer);
  sim.setTimestep(1.f / 200.f);
  auto builder = sim.createArticulationBuilder();
  auto link = builder->addLink(nullptr, {{0, 0, 0}, PxIdentity}, "body");
  builder->addSphereShapeToLink(*link);
  builder->updateLinkMassAndInertia(*link, 1000.f);

  builder->addSphereVisualToLink(*link);
  builder->build(false);

  sim.addGround(-2);

  while (true) {
    sim.step();
    sim.updateRenderer();
    renderer.render();
    if (Optifuser::getInput().getKeyState(GLFW_KEY_Q)) {
      break;
    }
  }
}
