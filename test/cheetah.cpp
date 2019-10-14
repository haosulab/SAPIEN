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

float rand_float(float min = -100, float max = 100) {
  float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  return (max - min) * r + min;
}

int main() {
  Renderer::OptifuserRenderer renderer;
  renderer.init();

  renderer.cam.position = {0, -1, 1};
  renderer.cam.rotateYawPitch(0, -0.5);

  Simulation sim;
  sim.setRenderer(&renderer);
  sim.setTimestep(1.f / 200.f);

  float inf = std::numeric_limits<float>::infinity();

  auto x2z = PxQuat(0, -0.7071068, 0, 0.7071068);
  auto x2y = PxQuat(0, 0, 0.7071068, 0.7071068);

  auto builder = sim.createArticulationBuilder();

  auto root1 = builder->addLink(nullptr, {{0, 0, 0}, PxIdentity}, "root1");
  auto root2 = builder->addLink(root1, {{0, 0, 0}, PxIdentity}, "root2", "fake1",
                                PxArticulationJointType::ePRISMATIC, {{-inf, inf}},
                                {{0, 0, 0}, x2z}, {{0, 0, 0}, x2z});
  auto root3 = builder->addLink(root2, {{0, 0, 0}, PxIdentity}, "root3", "fake2",
                                PxArticulationJointType::ePRISMATIC, {{-inf, inf}});
  auto torso = builder->addLink(root3, {{0, 0, 0}, PxIdentity}, "torso", "torso",
                                PxArticulationJointType::eREVOLUTE, {{-inf, inf}},
                                {{0, 0, 0}, x2y}, {{0, 0, 0}, x2y});

  builder->addCapsuleShapeToLink(*torso, {{0, 0, 0}, PxIdentity}, 0.046, .5);
  builder->addCapsuleVisualToLink(*torso, {{0, 0, 0}, PxIdentity}, 0.046, .5, {1, 1, 1}, "torso");

  builder->addCapsuleShapeToLink(*torso, {{.6, 0, .1}, PxQuat(.87, {0, 1, 0}) * x2z}, 0.046, .15);
  builder->addCapsuleVisualToLink(*torso, {{.6, 0, .1}, PxQuat(.87, {0, 1, 0}) * x2z}, 0.046, .15,
                                  {1, 1, 1}, "head");

  auto bthigh = builder->addLink(torso, {{0, 0, 0}, PxIdentity}, "bthigh", "bthigh",
                                 PxArticulationJointType::eREVOLUTE, {{-.52, 1.05}},
                                 {{-.5, 0, 0}, {0, 0, 0.7071068, 0.7071068}},
                                 {{0, 0, 0}, {0, 0, 0.7071068, 0.7071068}});
  builder->addCapsuleShapeToLink(*bthigh, {{.1, 0, -.13}, PxQuat(-3.8, {0, 1, 0}) * x2z}, 0.046,
                                 .145);
  builder->addCapsuleVisualToLink(*bthigh, {{.1, 0, -.13}, PxQuat(-3.8, {0, 1, 0}) * x2z}, 0.046,
                                  .145, {1, 1, 1}, "bthigh");

  auto bshin = builder->addLink(bthigh, {{0, 0, 0}, PxIdentity}, "bshin", "bshin",
                                PxArticulationJointType::eREVOLUTE, {{-.785, .785}},
                                {{.16, 0, -.25}, {0, 0, 0.7071068, 0.7071068}},
                                {{0, 0, 0}, {0, 0, 0.7071068, 0.7071068}});

  builder->addCapsuleShapeToLink(*bshin, {{-.14, 0, -.07}, PxQuat(-2.03, {0, 1, 0}) * x2z}, 0.046,
                                 .15);
  builder->addCapsuleVisualToLink(*bshin, {{-.14, 0, -.07}, PxQuat(-2.03, {0, 1, 0}) * x2z}, 0.046,
                                  .15, {.9, .6, .6}, "bshin");

  auto bfoot = builder->addLink(bshin, {{0, 0, 0}, PxIdentity}, "bfoot", "bfoot",
                                PxArticulationJointType::eREVOLUTE, {{-.4, .785}},
                                {{-.28, 0, -.14}, {0, 0, 0.7071068, 0.7071068}},
                                {{0, 0, 0}, {0, 0, 0.7071068, 0.7071068}});

  builder->addCapsuleShapeToLink(*bfoot, {{.03, 0, -.097}, PxQuat(-.27, {0, 1, 0}) * x2z}, 0.046,
                                 .094);
  builder->addCapsuleVisualToLink(*bfoot, {{.03, 0, -.097}, PxQuat(-.27, {0, 1, 0}) * x2z}, 0.046,
                                  .094, {.9, .6, .6}, "bfoot");

  auto fthigh = builder->addLink(torso, {{0, 0, 0}, PxIdentity}, "fthigh", "fthigh",
                                 PxArticulationJointType::eREVOLUTE, {{-1, .7}},
                                 {{.5, 0, 0}, {0, 0, 0.7071068, 0.7071068}},
                                 {{0, 0, 0}, {0, 0, 0.7071068, 0.7071068}});
  builder->addCapsuleShapeToLink(*fthigh, {{-.07, 0, -.12}, PxQuat(.52, {0, 1, 0}) * x2z}, 0.046,
                                 .133);
  builder->addCapsuleVisualToLink(*fthigh, {{-.07, 0, -.12}, PxQuat(.52, {0, 1, 0}) * x2z}, 0.046,
                                  .133, {1, 1, 1}, "fthigh");

  auto fshin = builder->addLink(fthigh, {{0, 0, 0}, PxIdentity}, "fshin", "fshin",
                                PxArticulationJointType::eREVOLUTE, {{-1.2, .87}},
                                {{-.14, 0, -.24}, {0, 0, 0.7071068, 0.7071068}},
                                {{0, 0, 0}, {0, 0, 0.7071068, 0.7071068}});
  builder->addCapsuleShapeToLink(*fshin, {{.065, 0, -.09}, PxQuat(-.6, {0, 1, 0}) * x2z}, 0.046,
                                 .106);
  builder->addCapsuleVisualToLink(*fshin, {{.065, 0, -.09}, PxQuat(-.6, {0, 1, 0}) * x2z}, 0.046,
                                  .106, {.9, .6, .6}, "fshin");

  auto ffoot = builder->addLink(fshin, {{0, 0, 0}, PxIdentity}, "ffoot", "ffoot",
                                PxArticulationJointType::eREVOLUTE, {{-.5, .5}},
                                {{.13, 0, -.18}, {0, 0, 0.7071068, 0.7071068}},
                                {{0, 0, 0}, {0, 0, 0.7071068, 0.7071068}});
  builder->addCapsuleShapeToLink(*ffoot, {{.045, 0, -.07}, PxQuat(-.6, {0, 1, 0}) * x2z}, 0.046,
                                 .07);
  builder->addCapsuleVisualToLink(*ffoot, {{.045, 0, -.07}, PxQuat(-.6, {0, 1, 0}) * x2z}, 0.046,
                                  .07, {.9, .6, .6}, "ffoot");

  auto wrapper = builder->build(true);

  sim.addGround(-1);

  std::vector<std::array<float, 2>> coords = {{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {0, 4},
                                              {1, 4}, {2, 4}, {3, 4}, {4, 4}, {0, 1}, {0, 2},
                                              {0, 3}, {4, 1}, {4, 2}, {4, 3}};
  // for (auto &[x, y] : coords) {
  //   auto actor_builder = sim.createActorBuilder();
  //   actor_builder->addBoxShape({{0, 0, 0}, PxIdentity}, {0.5, 0.5, 1});
  //   actor_builder->addBoxVisual({{0, 0, 0}, PxIdentity}, {0.5, 0.5, 1});
  //   actor_builder->build(true)->setGlobalPose({{x-1, y-1, 0}, PxIdentity});
  // }

  wrapper->set_qpos({0, 0, 0, 0, 0, 0, 0, 0, 0});
  wrapper->set_qvel({0, 0, 0, 0, 0, 0, 0, 0, 0});
  wrapper->set_qf({0, 0, 0, 0, 0, 0, 0, 0, 0});

  renderer.showWindow();
  while (true) {
    wrapper->set_qf({0, 0, 0, rand_float(), rand_float(), rand_float(), rand_float(), rand_float(),
                     rand_float()});
    for (int i = 0; i < 10; ++i) {
      sim.step();
    }
    sim.updateRenderer();
    renderer.render();
    if (Optifuser::getInput().getKeyState(GLFW_KEY_Q)) {
      break;
    }
  }
}
