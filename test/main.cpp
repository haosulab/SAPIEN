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

void test1() {
  Renderer::OptifuserRenderer renderer;

  Simulation sim;
  sim.setRenderer(&renderer);
  sim.setTimestep(1.f / 60.f);

  auto builder = sim.createActorBuilder();
  for (const auto entry :
       fs::directory_iterator("/home/fx/mobility_mesh/resources/46437-4/objs")) {
    builder->addConvexShapeFromObj(entry.path());
    builder->addObjVisual(entry.path());
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
  Renderer::OptifuserRenderer renderer;

  Simulation sim;
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

  ArticulationWrapper *articulationWrapper = builder->build(true);
  auto articulation = articulationWrapper->articulation;
  auto cache = articulationWrapper->cache;

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

void test3() {
  Renderer::OptifuserRenderer renderer("130");

  renderer.cam.position = {0, -2, 3};
  renderer.cam.setForward({0, 1, 0});
  renderer.cam.setUp({0, 0, 1});
  renderer.cam.rotateYawPitch(0, -0.5);

  std::array<float, 3> color = {1 * 4, 0.773 * 4, 0.561 * 4};
  renderer.addPointLight({0, 0, 4}, color);
  renderer.addPointLight({0, -4, 4}, color);
  renderer.addPointLight({0, -2, 4}, color);

  Simulation sim;
  sim.setRenderer(&renderer);
  sim.setTimestep(1.f / 500.f);

  auto ab = sim.createActorBuilder();
  ab->addMultipleConvexShapesFromObj("../assets/object/walls/wall_scene.obj");
  ab->addObjVisual("../assets/object/walls/wall_scene.obj");
  ab->build(true, false, "Scene");

  auto loader = URDF::URDFLoader(sim);
  loader.fixLoadedObject = false;

  // auto obj = loader.load("../assets/179/mobility.urdf");
  // obj->articulation->teleportRootLink({0, 0, 1}, true);

  // auto obj = loader.load("../assets/robot/sapien_gripper.urdf");
  auto obj = loader.load("../assets/robot/movo_sapien.urdf");
  obj->articulation->teleportRootLink({0, 0, 1}, true);

  // auto obj1 = loader.loadKinematic(
  //     "/home/fx/source/partnet-mobility-scripts/mobility_v1_alpha5/102044/mobility.urdf");
  // obj1->articulation->teleportRootLink({1.221, 1.244, 0.614}, true);

  // loader.scale = 0.5;
  // auto obj2 = loader.load(
  //     "/home/fx/source/partnet-mobility-scripts/mobility_v1_alpha5/102611/mobility.urdf");
  // obj2->articulation->teleportRootLink({0.900, -0.954, 0.622}, true);

  // loader.scale = 0.8;
  // auto obj3 = loader.load(
  //     "/home/fx/source/partnet-mobility-scripts/mobility_v1_alpha5/12065/mobility.urdf");
  // obj3->articulation->teleportRootLink({1.12, 0.109, 0.582}, true);

  // loader.scale = 1;
  // auto obj4 = loader.load(
  //     "/home/fx/source/partnet-mobility-scripts/mobility_v1_alpha5/23511/mobility.urdf");
  // obj4->articulation->teleportRootLink({{-2.246, -3.518, 0.910}, PxQuat(3.14159, {0, 0,
  // 1})}, true);

  // loader.scale = 1;
  // auto obj5 = loader.load(
  //     "/home/fx/source/partnet-mobility-scripts/mobility_v1_alpha5/45594/mobility.urdf");
  // obj5->articulation->teleportRootLink({1.271, 2.393, 0.946}, true);

  // loader.scale = 1;
  // auto obj6 = loader.load(
  //     "/home/fx/source/partnet-mobility-scripts/mobility_v1_alpha5/46037/mobility.urdf");
  // obj6->articulation->teleportRootLink({{0.597, -3.789, 0.774}, PxQuat(-1.5708, {0, 0, 1})},
  // true);

  // loader.scale = 0.4;
  // auto obj7 = loader.load(
  //     "/home/fx/source/partnet-mobility-scripts/mobility_v1_alpha5/7310/mobility.urdf");
  // obj7->articulation->teleportRootLink({{1.195, 0.847, 1.259}, PxQuat(-0.1, {0, 0, 1})},
  // true);

  // loader.scale = 1.5;
  // auto obj8 = loader.load(
  //     "/home/fx/source/partnet-mobility-scripts/mobility_v1_alpha5/8867/mobility.urdf");
  // obj8->articulation->teleportRootLink({-3.127, 2.281, 1.481}, true);

  // std::ifstream s("/home/fx/source/partnet-mobility-scripts/46627/nocs.txt");
  // std::string line;
  // while (std::getline(s, line)) {
  //   if (line.length() == 0 || line[0] == ' ')
  //     continue;
  //   std::istringstream is(line);
  //   std::string name;
  //   std::vector<float> mat(16);
  //   is >> name;
  //   std::cout << name << std::endl;
  //   is >> mat[0] >> mat[4] >> mat[8] >> mat[12] >> mat[1] >> mat[5] >> mat[9] >> mat[13] >>
  //       mat[2] >> mat[6] >> mat[10] >> mat[14] >> mat[3] >> mat[7] >> mat[11] >> mat[15];
  //   size_t idx = std::find(names.begin(), names.end(), name) - names.begin();
  //   std::cout << idx << std::endl;
  //   if (idx < names.size()) {
  //     renderer.setSegmentationCustomData(linkIds[idx], mat);
  //   }
  // }

  // for (auto n : names) {
  //   std::cout  <<  n << " ";
  // }
  // std::cout << std::endl;

  // int row = 2;
  // for (int i = 0; i < 4; ++i) {
  //   int x = i / row * 2;
  //   int y = i % row * 2;
  // auto articulationWrapper = loader.load("../assets/robot/all_robot.urdf");
  // articulationWrapper->articulation->teleportRootLink({{-1,0,-.5}, PxIdentity}, true);
  //   auto articulation = articulationWrapper->articulation;
  //   articulation->teleportRootLink({{(float)x, (float)y, 0}, PxQuat(rand_float()*3, {0, 0,
  //   1})}, true); articulationWrapper->updateCache();
  // }

  // auto chair = loader.load("../assets/179/test.urdf");
  // chair->articulation->teleportRootLink({{1,0,0}, PxIdentity}, true);

  // auto *articulationWrapper = loader.load("../assets/robot/all_robot.urdf");

  // auto *articulation = articulationWrapper->articulation;
  // auto *cache = articulationWrapper->cache;

  // sim.addGround(-1);

  // reset(articulationWrapper);
  // articulationWrapper->set_qpos({0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});

  // PxArticulationLink *chairLink;
  // chair->articulation->getLinks(&chairLink, 1);

  // auto actorBuider = sim.createActorBuilder();
  // auto actor = actorBuider->build(false, true, "Camera Mount");
  // sim.addMountedCamera("Floating Camera", actor, {{0, 0, 0}, PxIdentity}, 256, 256, 0.9,
  // 0.9); actor->setGlobalPose({{-10, 0, 1}, {0, 0, 0, 1}}); actor->setGlobalPose({{-2, 0, 2},
  // {0, 0.3826834, 0, 0.9238795}});

  // auto names = wrapper->get_link_names();

  // std::vector<float> ps(wrapper->dof());
  // physx::PxVec3 vec = {1, 0, 0};
  // wrapper->set_qpos(ps);
  // auto pose = wrapper->get_link_joint_pose(1);
  // physx::PxVec3 newvec = pose.rotate(vec);
  // std::cout << names[1] << newvec.x << " " << newvec.y << " " << newvec.z << std::endl;

  // pose = wrapper->get_link_joint_pose(2);
  // newvec = pose.rotate(vec);
  // std::cout << names[2] << newvec.x << " " << newvec.y << " " << newvec.z << std::endl;

  // pose = wrapper->get_link_joint_pose(3);
  // newvec = pose.rotate(vec);
  // std::cout << names[3] << newvec.x << " " << newvec.y << " " << newvec.z << std::endl;

  // pose = wrapper->get_link_joint_pose(4);
  // newvec = pose.rotate(vec);
  // std::cout << names[4] << newvec.x << " " << newvec.y << " " << newvec.z << std::endl;

  printf("Simulation start\n");
  while (true) {
    // articulation->commonInit();
    // for (uint32_t i = 0; i < 13; ++i) {
    //   cache->jointAcceleration[i] = 1;
    // }

    // articulation->computeJointForce(*cache);
    // articulation->applyCache(*cache, PxArticulationCache::eFORCE);

    sim.step();
    sim.updateRenderer();
    renderer.render();
    renderer.showWindow();

    if (Optifuser::getInput().getKeyState(GLFW_KEY_Q)) {
      break;
    }
  }
}


int main(int argc, char **argv) { test3(); }
