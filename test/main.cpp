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
#include <experimental/filesystem>
#include "optifuser_renderer.h"
#include "simulation.h"
#include "actor_builder.h"

#include <random>

using namespace physx;
namespace fs = std::experimental::filesystem;

int main(int argc, char **argv) {
  OptifuserRenderer renderer;
  renderer.init();

  PxSimulation sim;
  sim.setRenderer(&renderer);
  sim.setTimestep(1.f / 60.f);

  auto builder = sim.createActorBuilder();
  for (const auto entry : fs::directory_iterator("/home/fx/mobility_mesh/resources/46437-4/objs")) {
    builder->addConvexShapeFromObj(entry.path(), entry.path());
  }
  builder->build()
      ->setGlobalPose(PxTransform({0, 1, 0}, PxIdentity));
  builder = sim.createActorBuilder();
  builder->addPrimitiveShape(physx::PxGeometryType::ePLANE);
  builder->build(true)
      ->setGlobalPose(PxTransform({0,0,0}, PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f))));

  auto mesh = Optifuser::NewMeshGrid();
  mesh->position = {0,0.001,0};
  renderer.mScene->addObject(mesh);

  while (true) {
    sim.step();
    sim.updateRenderer();
    renderer.render();
    if (Optifuser::getInput().getKeyState(GLFW_KEY_Q)) {
      break;
    }
  }
}
