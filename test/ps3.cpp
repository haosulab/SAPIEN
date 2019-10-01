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

#include "input/ps3.hpp"
#include <random>

using namespace PS3;
using namespace physx;
namespace fs = std::experimental::filesystem;

void reset(PxArticulationWrapper *wrapper) {
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
  // PS3Input ps3;

  Renderer::OptifuserRenderer renderer;
  renderer.init();
  renderer.cam.position = {0, -2, 0.5};
  renderer.cam.setForward({0, 1, 0});
  renderer.cam.setUp({0, 0, 1});

  PxSimulation sim;
  sim.setRenderer(&renderer);
  sim.setTimestep(1.f / 500.f);

  sim.addGround(0);

  auto builder = sim.createActorBuilder();
  builder->addBoxShape({{0, 0, 0}, PxIdentity}, {0.3, 0.3, 0.3});
  builder->addBoxVisual({{0, 0, 0}, PxIdentity}, {0.3, 0.3, 0.3});
  auto actor = builder->build(false, false, false);
  actor->setGlobalPose({{2, 2, 1}, PxIdentity});
  sim.mScene->addActor(*actor);

  auto loader = URDF::URDFLoader(sim);
  auto *articulationWrapper = loader.load("../assets/robot/all_robot.urdf");
  auto articulation = articulationWrapper->articulation;

  // auto cache = articulationInfo.cache;
  printf("dof: %d\n", articulation->getDofs());

  sim.step();
  reset(articulationWrapper);
  articulationWrapper->updateArticulation();

  PxVec3 pos = {0, 0, 0};
  PxReal angle = 0;

  printf("Simulation start\n");
  while (true) {
    // if (ps3.getKey(BUTTON_UP)) {
    //   pos += PxQuat(angle, {0, 0, 1}).rotate(PxVec3(1, 0, 0)) * 0.01;
    // } else if (ps3.getKey(BUTTON_DOWN)) {
    //   pos -= PxQuat(angle, {0, 0, 1}).rotate(PxVec3(1, 0, 0)) * 0.01;
    // }
    // if (ps3.getKey(BUTTON_LEFT)) {
    //   pos += PxQuat(angle, {0, 0, 1}).rotate(PxVec3(0, 1, 0)) * 0.01;
    // } else if (ps3.getKey(BUTTON_RIGHT)) {
    //   pos -= PxQuat(angle, {0, 0, 1}).rotate(PxVec3(0, 1, 0)) * 0.01;
    // }
    // if (ps3.getKey(BUTTON_L2)) {
    //   angle += 0.01;
    // } else if (ps3.getKey(BUTTON_R2)) {
    //   angle -= 0.01;
    // }

    sim.step();
    articulation->teleportRootLink({pos, PxQuat(angle, {0, 0, 1})}, true);
    articulationWrapper->updateCache();

    sim.updateRenderer();
    renderer.render();
    if (Optifuser::getInput().getKeyState(GLFW_KEY_Q)) {
      break;
    }
    // if (ps3.getKey(BUTTON_X)) {
    //   break;
    // }
  }
}

int main(int argc, char **argv) { test(); }
