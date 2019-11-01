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

using namespace physx;
namespace fs = std::experimental::filesystem;
using namespace sapien;

void reset(ArticulationWrapper *wrapper) {
  wrapper->articulation->copyInternalStateToCache(*wrapper->cache, PxArticulationCache::eALL);
  for (size_t i = 0; i < wrapper->articulation->getDofs(); ++i) {
    wrapper->cache->jointPosition[i] = 0;
  }
  wrapper->articulation->applyCache(*wrapper->cache, PxArticulationCache::eALL);
}

float rand_float() {
  float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  return 2 * r - 1;
}

void test() {
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
  auto wrapper = loader.loadKinematic("../assets/7619/mobility.urdf");

  auto actorBuider = sim.createActorBuilder();
  auto actor = actorBuider->build(false, true, "Camera Mount");
  sim.addMountedCamera("Floating Camera", actor, {{0, 0, 0}, PxIdentity}, 256, 256, 0.9, 0.9);
  actor->setGlobalPose({{-10, 0, 1}, {0, 0, 0, 1}});
  actor->setGlobalPose({{-2, 0, 2}, {0, 0.3826834, 0, 0.9238795}});

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

int main(int argc, char **argv) { test(); }
