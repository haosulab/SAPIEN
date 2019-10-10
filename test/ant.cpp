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

float rand_float(float min = -6000, float max = 6000) {
  float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  return (max - min) * r + min;
}

int main() {
  Renderer::OptifuserRenderer renderer;
  renderer.init();

  renderer.cam.position = {0, 1, 10};
  renderer.cam.setForward({0, 1, 0});
  renderer.cam.setUp({0, 0, 1});
  renderer.cam.rotateYawPitch(0, -1.5);

  Simulation sim;
  sim.setRenderer(&renderer);
  sim.setTimestep(1.f / 200.f);
  auto builder = sim.createArticulationBuilder();
  auto body_link = builder->addLink(nullptr, {{0, 0, 0}, PxIdentity}, "body");

  builder->addSphereShapeToLink(*body_link, {{0, 0, 0}, PxIdentity}, 0.25);
  builder->addSphereVisualToLink(*body_link, {{0, 0, 0}, PxIdentity}, 0.25);

  builder->addCapsuleShapeToLink(*body_link, {{0.141, 0, 0}, PxIdentity}, 0.08, 0.141);
  builder->addCapsuleVisualToLink(*body_link, {{0.141, 0, 0}, PxIdentity}, 0.08, 0.141);

  builder->addCapsuleShapeToLink(*body_link, {{-0.141, 0, 0}, PxIdentity}, 0.08, 0.141);
  builder->addCapsuleVisualToLink(*body_link, {{-0.141, 0, 0}, PxIdentity}, 0.08, 0.141);

  builder->addCapsuleShapeToLink(
      *body_link, {{0, 0.141, 0}, physx::PxQuat(M_PIf32 / 2, {0, 0, 1})}, 0.08, 0.141);
  builder->addCapsuleVisualToLink(
      *body_link, {{0, 0.141, 0}, physx::PxQuat(M_PIf32 / 2, {0, 0, 1})}, 0.08, 0.141);
  builder->addCapsuleShapeToLink(
      *body_link, {{0, -0.141, 0}, physx::PxQuat(M_PIf32 / 2, {0, 0, 1})}, 0.08, 0.141);
  builder->addCapsuleVisualToLink(
      *body_link, {{0, -0.141, 0}, physx::PxQuat(M_PIf32 / 2, {0, 0, 1})}, 0.08, 0.141);
  builder->updateLinkMassAndInertia(*body_link, 1000.f);

  auto l1 = builder->addLink(body_link, {{0, 0, 0}, PxIdentity}, "l1", "j1",
                             PxArticulationJointType::eREVOLUTE, {{-0.5236, 0.5236}},
                             {{0.282, 0, 0}, {0, 0.7071068, 0, 0.7071068}},
                             {{0.141, 0, 0}, {0, -0.7071068, 0, 0.7071068}});
  builder->addCapsuleShapeToLink(*l1, {{0, 0, 0}, PxIdentity}, 0.08, 0.141);
  builder->addCapsuleVisualToLink(*l1, {{0, 0, 0}, PxIdentity}, 0.08, 0.141);
  builder->updateLinkMassAndInertia(*l1, 1000.f);

  auto l2 = builder->addLink(body_link, {{0, 0, 0}, PxIdentity}, "l2", "j2",
                             PxArticulationJointType::eREVOLUTE, {{-0.5236, 0.5236}},
                             {{-0.282, 0, 0}, {0.7071068, 0, -0.7071068, 0}},
                             {{0.141, 0, 0}, {0, -0.7071068, 0, 0.7071068}});
  builder->addCapsuleShapeToLink(*l2, {{0, 0, 0}, PxIdentity}, 0.08, 0.141);
  builder->addCapsuleVisualToLink(*l2, {{0, 0, 0}, PxIdentity}, 0.08, 0.141);
  builder->updateLinkMassAndInertia(*l2, 1000.f);

  auto l3 = builder->addLink(body_link, {{0, 0, 0}, PxIdentity}, "l3", "j3",
                             PxArticulationJointType::eREVOLUTE, {{-0.5236, 0.5236}},
                             {{0, 0.282, 0}, {-0.5, 0.5, 0.5, 0.5}},
                             {{0.141, 0, 0}, {0, -0.7071068, 0, 0.7071068}});
  builder->addCapsuleShapeToLink(*l3, {{0, 0, 0}, PxIdentity}, 0.08, 0.141);
  builder->addCapsuleVisualToLink(*l3, {{0, 0, 0}, PxIdentity}, 0.08, 0.141);
  builder->updateLinkMassAndInertia(*l3, 1000.f);

  auto l4 = builder->addLink(body_link, {{0, 0, 0}, PxIdentity}, "l4", "j4",
                             PxArticulationJointType::eREVOLUTE, {{-0.5236, 0.5236}},
                             {{0, -0.282, 0}, {0.5, 0.5, -0.5, 0.5}},
                             {{0.141, 0, 0}, {0, -0.7071068, 0, 0.7071068}});
  builder->addCapsuleShapeToLink(*l4, {{0, 0, 0}, PxIdentity}, 0.08, 0.141);
  builder->addCapsuleVisualToLink(*l4, {{0, 0, 0}, PxIdentity}, 0.08, 0.141);
  builder->updateLinkMassAndInertia(*l4, 1000.f);

  auto f1 = builder->addLink(l1, {{0, 0, 0}, PxIdentity}, "f1", "j11",
                             PxArticulationJointType::eREVOLUTE, {{0.5236, 1.222}},
                             {{-0.141, 0, 0}, {0.7071068, 0.7071068, 0, 0}},
                             {{0.282, 0, 0}, {0.7071068, 0.7071068, 0, 0}});
  builder->addCapsuleShapeToLink(*f1, {{0, 0, 0}, PxIdentity}, 0.08, 0.282);
  builder->addCapsuleVisualToLink(*f1, {{0, 0, 0}, PxIdentity}, 0.08, 0.282);
  builder->updateLinkMassAndInertia(*f1, 1000.f);

  auto f2 = builder->addLink(l2, {{0, 0, 0}, PxIdentity}, "f2", "j21",
                             PxArticulationJointType::eREVOLUTE, {{0.5236, 1.222}},
                             {{-0.141, 0, 0}, {0.7071068, 0.7071068, 0, 0}},
                             {{0.282, 0, 0}, {0.7071068, 0.7071068, 0, 0}});
  builder->addCapsuleShapeToLink(*f2, {{0, 0, 0}, PxIdentity}, 0.08, 0.282);
  builder->addCapsuleVisualToLink(*f2, {{0, 0, 0}, PxIdentity}, 0.08, 0.282);
  builder->updateLinkMassAndInertia(*f2, 1000.f);

  auto f3 = builder->addLink(l3, {{0, 0, 0}, PxIdentity}, "f3", "j31",
                             PxArticulationJointType::eREVOLUTE, {{0.5236, 1.222}},
                             {{-0.141, 0, 0}, {0.7071068, 0.7071068, 0, 0}},
                             {{0.282, 0, 0}, {0.7071068, 0.7071068, 0, 0}});
  builder->addCapsuleShapeToLink(*f3, {{0, 0, 0}, PxIdentity}, 0.08, 0.282);
  builder->addCapsuleVisualToLink(*f3, {{0, 0, 0}, PxIdentity}, 0.08, 0.282);
  builder->updateLinkMassAndInertia(*f3, 1000.f);

  auto f4 = builder->addLink(l4, {{0, 0, 0}, PxIdentity}, "f4", "j41",
                             PxArticulationJointType::eREVOLUTE, {{0.5236, 1.222}},
                             {{-0.141, 0, 0}, {0.7071068, 0.7071068, 0, 0}},
                             {{0.282, 0, 0}, {0.7071068, 0.7071068, 0, 0}});
  builder->addCapsuleShapeToLink(*f4, {{0, 0, 0}, PxIdentity}, 0.08, 0.282);
  builder->addCapsuleVisualToLink(*f4, {{0, 0, 0}, PxIdentity}, 0.08, 0.282);
  builder->updateLinkMassAndInertia(*f4, 1000.f);

  auto wrapper = builder->build(false);

  sim.addGround(-1);

  std::vector<std::array<float, 2>> coords = {{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {0, 4},
                                              {1, 4}, {2, 4}, {3, 4}, {4, 4}, {0, 1}, {0, 2},
                                              {0, 3}, {4, 1}, {4, 2}, {4, 3}};
  for (auto &[x, y] : coords) {
    auto actor_builder = sim.createActorBuilder();
    actor_builder->addBoxShape({{0, 0, 0}, PxIdentity}, {0.5, 0.5, 1});
    actor_builder->addBoxVisual({{0, 0, 0}, PxIdentity}, {0.5, 0.5, 1});
    actor_builder->build(true)->setGlobalPose({{x-1, y-1, 0}, PxIdentity});
  }

  renderer.showWindow();
  while (true) {
    // wrapper->set_qf({rand_float(), rand_float(), rand_float(), rand_float(), rand_float(),
    //                  rand_float(), rand_float(), rand_float()});
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
