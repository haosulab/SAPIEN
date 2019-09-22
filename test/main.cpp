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

void test1() {
  OptifuserRenderer renderer;
  renderer.init();

  PxSimulation sim;
  sim.setRenderer(&renderer);
  sim.setTimestep(1.f / 60.f);

  auto builder = sim.createActorBuilder();
  for (const auto entry :
       fs::directory_iterator("/home/fx/mobility_mesh/resources/46437-4/objs")) {
    builder->addConvexShapeFromObj(entry.path(), entry.path());
  }
  builder->build(true)->setGlobalPose(PxTransform({0, 1, 0}, PxIdentity));

  while (true) {
    sim.step();
    sim.updateRenderer();
    renderer.render();
    if (Optifuser::getInput().getKeyState(GLFW_KEY_Q)) {
      break;
    }
  }
}

void test2() {
  OptifuserRenderer renderer;
  renderer.init();

  PxSimulation sim;
  sim.setRenderer(&renderer);
  sim.setTimestep(1.f / 60.f);

  auto builder = sim.createArticulationBuilder();
  auto link1 = builder->addLink(nullptr);
  builder->addBoxShapeToLink(*link1);
  builder->addBoxVisualToLink(*link1);
  builder->updateLinkMassAndInertia(*link1);

  auto link2 = builder->addLink(link1);
  builder->addBoxShapeToLink(*link2);
  builder->addBoxVisualToLink(*link2);
  builder->updateLinkMassAndInertia(*link2);

  auto joint = static_cast<PxArticulationJointReducedCoordinate *>(link2->getInboundJoint());
  joint->setJointType(PxArticulationJointType::eREVOLUTE);
  joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
  joint->setParentPose({{0, 0, 0}, PxIdentity});
  joint->setChildPose({{0, 3, 0}, PxIdentity});

  auto articulationInfo = builder->build(true);
  auto articulation = articulationInfo.articulation;
  auto cache = articulationInfo.cache;

  auto mesh = Optifuser::NewMeshGrid();
  mesh->position = {0, 0.001, 0};
  renderer.mScene->addObject(std::move(mesh));

  sim.step();
  while (true) {
    sim.step();
    sim.updateRenderer();
    articulation->copyInternalStateToCache(*cache, PxArticulationCache::eALL);
    std::cout << cache->jointPosition[0] << std::endl;
    renderer.render();
    if (Optifuser::getInput().getKeyState(GLFW_KEY_Q)) {
      break;
    }
  }
}

void reset(PxArticulationWrapper &info) {
  info.articulation->copyInternalStateToCache(*info.cache, PxArticulationCache::eALL);
  for (size_t i = 0; i < info.articulation->getDofs(); ++i) {
    info.cache->jointPosition[i] = 0;
  }
  info.articulation->applyCache(*info.cache, PxArticulationCache::eALL);
}

float rand_float() {
  float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  return 2 * r - 1;
}

void test3() {
  OptifuserRenderer renderer;
  renderer.init();
  renderer.cam.position = {0, -2, 0.5};
  renderer.cam.forward = {0, 1, 0};
  renderer.cam.up = {0, 0, 1};

  PxSimulation sim;
  sim.setRenderer(&renderer);
  sim.setTimestep(1.f / 500.f);

  auto loader = URDFLoader(sim);
  auto &articulationInfo = loader.load("../assets/robot/all_robot.urdf");
  auto articulation = articulationInfo.articulation;

  // auto cache = articulationInfo.cache;
  printf("dof: %d\n", articulation->getDofs());

  sim.step();
  reset(articulationInfo);
  articulationInfo.set_qpos_unchecked({0.472});
  // articulationInfo.set_qpos_unchecked({0,0,0,-1.93475823254, -1.53188487338, 0.951243371548,
  //                                      -2.24179359416, 0.344180286477, 0.649430580507,
  //                                      -1.41300076449});

  articulationInfo.set_qpos_unchecked({0,0,0,0,0,0,0,0,0,0,0,0,0});
  articulationInfo.set_qvel_unchecked({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  std::cout << articulationInfo.summary() << std::endl;
  articulationInfo.updateArticulation();

  printf("Simulation start\n");
  while (true) {
    sim.step();
    sim.updateRenderer();
    articulationInfo.updateCache();
    renderer.render();
    if (Optifuser::getInput().getKeyState(GLFW_KEY_Q)) {
      break;
    }
  }
}

int main(int argc, char **argv) { test3(); }
