//
// Created by sim on 11/12/19.
//
#include "actor_builder.h"
#include "articulation_builder.h"
#include "controller/controller_manger.h"
#include "optifuser_renderer.h"
#include "simulation.h"
#include "urdf/urdf_loader.h"
#include <device/movo.hpp>
#include <device/single_gripper.hpp>
#include <extensions/PxDefaultAllocator.h>
#include <foundation/PxMat33.h>
#include <optifuser.h>
#include <thread>

using namespace sapien;
void test3() {
  Renderer::OptifuserRenderer renderer;

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

  float global_scale = 1;

  auto ab = sim.createActorBuilder();
  ab->addMultipleConvexShapesFromObj("../assets/object/walls/wall_scene.obj",
                                     {{0, 0, 0}, PxIdentity},
                                     {global_scale, global_scale, global_scale});
  ab->addObjVisual("../assets/object/walls/wall_scene.obj", {{0, 0, 0}, PxIdentity},
                   {global_scale, global_scale, global_scale});
  ab->build(true, false, "Scene");

  auto loader = URDF::URDFLoader(sim);
  loader.fixLoadedObject = true;
  loader.scale = 1.f * global_scale;
  auto obj1 = loader.load("/home/sim/mobility_dataset/mobility_v1_alpha5/102044/mobility.urdf");
  obj1->articulation->teleportRootLink(
      {1.221f * global_scale, 1.244f * global_scale, 0.614f * global_scale}, true);

  loader.scale = 1.4 * global_scale;
  auto obj2 = loader.load("/home/sim/mobility_dataset/mobility_v1_alpha5/10905/mobility.urdf");
  obj2->articulation->teleportRootLink(
      {0.900f * global_scale, -0.954f * global_scale, 0.622f * global_scale}, true);

  loader.scale = 0.8 * global_scale;
  auto obj3 = loader.load("/home/sim/mobility_dataset/mobility_v1_alpha5/12065/mobility.urdf");
  obj3->articulation->teleportRootLink(
      {1.12f * global_scale, 0.109f * global_scale, 0.582f * global_scale}, true);
  obj3->set_drive_property(20000, 3000, 2000, {1});

  loader.scale = 1 * global_scale;
  auto obj4 = loader.load("/home/sim/mobility_dataset/mobility_v1_alpha5/23511/mobility.urdf");
  obj4->articulation->teleportRootLink(
      {{-2.246f * global_scale, -3.518f * global_scale, 0.910f * global_scale},
       PxQuat(3.14159, {0, 0, 1})},
      true);

  loader.scale = 1 * global_scale;
  auto obj5 = loader.load("/home/sim/mobility_dataset/mobility_v1_alpha5/45594/mobility.urdf");
  obj5->articulation->teleportRootLink(
      {1.271f * global_scale, 2.393f * global_scale, 0.946f * global_scale}, true);

  loader.scale = 1 * global_scale;
  auto obj6 = loader.load("/home/sim/mobility_dataset/mobility_v1_alpha5/46037/mobility.urdf");
  obj6->articulation->teleportRootLink(
      {{0.597f * global_scale, -3.789f * global_scale, 0.774f * global_scale},
       PxQuat(-1.5708, {0, 0, 1})},
      true);

  loader.scale = 0.4 * global_scale;
  auto obj7 = loader.load("/home/sim/mobility_dataset/mobility_v1_alpha5/7310/mobility.urdf");
  obj7->articulation->teleportRootLink(
      {{1.195f * global_scale, 0.847f * global_scale, 1.259f * global_scale},
       PxQuat(-0.1, {0, 0, 1})},
      true);

  loader.scale = 1.5 * global_scale;
  auto obj8 = loader.load("/home/sim/mobility_dataset/mobility_v1_alpha5/8867/mobility.urdf");
  obj8->articulation->teleportRootLink(
      {-3.127f * global_scale, 2.281f * global_scale, 1.481f * global_scale}, true);

  loader.balancePassiveForce = true;
  loader.scale = 1;
  //  float pid_scale = 8;
  auto gripperMaterial = sim.mPhysicsSDK->createMaterial(3, 2, 0.01);
  auto wrapper = loader.load("../assets/robot/all_robot.urdf", gripperMaterial);
  wrapper->set_drive_property(20000, 3000, 2000, {0});
  wrapper->set_drive_property(2000, 300, 300, {1, 3, 5, 6, 7, 8, 9});
  wrapper->set_drive_property(500, 100, 300, {2, 4});
  wrapper->set_drive_property(200, 40, 20, {10, 11, 12});
  std::vector<float> init_qpos = {0.25, -1.9347, 0,      -1.5318, 0, 0.9512, -2.24,
                                  0.34, 0.64,    -1.413, 0,       0, 0};
  wrapper->set_qpos(init_qpos);
  wrapper->set_drive_target(init_qpos);
  wrapper->move_base({-0.3, 0, 0.05});

  const std::vector<std::string> gripperJoints = {
      "right_gripper_finger1_joint", "right_gripper_finger2_joint", "right_gripper_finger3_joint"};

  const std::vector<std::string> headJoints = {"pan_joint", "tilt_joint"};
  const std::vector<std::string> bodyJoints = {"linear_joint"};

  auto controllableWrapper = sim.createControllableArticulationWrapper(wrapper);
  auto manger = std::make_unique<robot::ControllerManger>("movo", controllableWrapper);
  manger->createJointPubNode(100);
  manger->createJointVelocityController(gripperJoints, "right_gripper");
  manger->createCartesianVelocityController("right_arm");
  manger->createJointVelocityController(headJoints, "head");
  manger->createJointVelocityController(bodyJoints, "body");
  manger->start();
  robot::MOVOPS3 ps3(manger.get());
//  ps3.set_translation_velocity(2);
//  ps3.set_rotation_velocity(5);
  ps3.set_gripper_velocity(10);

  printf("Simulation start\n");
  while (true) {
    sim.step();
    sim.updateRenderer();
    ps3.step();
    renderer.render();
    renderer.showWindow();

    if (Optifuser::getInput().getKeyState(GLFW_KEY_Q)) {
      break;
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ps3_movo");
  test3();
}
