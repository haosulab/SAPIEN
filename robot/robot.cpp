//
// Created by sim on 9/27/19.
//

#include "actor_builder.h"
#include "articulation_builder.h"
#include "joint_pub_node.h"
#include "optifuser_renderer.h"
#include "simulation.h"
#include "urdf/urdf_loader.h"
#include <PxPhysicsAPI.h>
#include <experimental/filesystem>
#include <extensions/PxDefaultAllocator.h>
#include <extensions/PxDefaultCpuDispatcher.h>
#include <extensions/PxSimpleFactory.h>
#include <foundation/PxMat33.h>
#include <iostream>
#include <joint_pub_node.h>
#include <object.h>
#include <optifuser.h>
#include <thread>
#include <vector>

void test1() {
  Renderer::OptifuserRenderer renderer;
  renderer.init();
  renderer.cam.position = {0.5, -4, 0.5};
  renderer.cam.setForward({0, 1, 0});
  renderer.cam.setUp({0, 0, 1});

  PxSimulation sim;
  sim.setRenderer(&renderer);
  sim.setTimestep(1.f / 60.f);

  auto builder = sim.createActorBuilder();
  PxReal boxLength = 0.5f;
  // PxVec3 boxSize = {0.3, 0.3, 0.3};
  PxVec3 boxSize = {boxLength, boxLength, boxLength};
  PxTransform boxPose({0.5f + boxLength, 0.f, -0.5f + boxLength + 1.f}, PxIdentity);
  builder->addBoxShape(boxPose, boxSize, nullptr, 1.f);
  builder->addBoxVisual(boxPose, boxSize);
  auto box = builder->build();
//  sim.addGround(-0.3f);

  auto loader = URDF::URDFLoader(sim);
  std::unique_ptr<PxKinematicsArticulationWrapper> unique_wrapper =
      loader.loadKinematic("../assets/robot/arm_without_gazabo.urdf");
  auto wrapper = unique_wrapper.get();
  sim.mKinematicArticulationWrappers.push_back(std::move(unique_wrapper));

  // ROS
  auto nh = std::make_shared<ros::NodeHandle>();
  robot_interface::JointPubNode node(wrapper->get_queue(), wrapper->get_drive_joint_name(), 30,
                                     1000, "/joint_states", nh);
  std::thread th(&robot_interface::JointPubNode::spin, &node);

  std::vector<PxReal> initQpos = {-1.93475823254,
                                  -1.53188487338,
                                  0.951243371548,
                                  -2.24179359416,
                                  0.344180286477,
                                  0.649430580507,
                                  -1.41300076449,
                                  0,
                                  0,
                                  0};
  wrapper->set_qpos(initQpos);
  wrapper->set_qvel({0.1, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0, 0, 0});
  bool simulating = false;
  sim.step();
  sim.updateRenderer();
  sim.step();
  sim.updateRenderer();
  //  wrapper->set_drive_target({0,0,0,0,0,0,0,0,0,0});
  //  wrapper->set_qvel({0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.02,0.02,0.02});

  while (true) {
    if (simulating) {
      sim.step();
      sim.updateRenderer();
    }
    renderer.render();
    auto input = Optifuser::getInput();
    if (input.getKeyState(GLFW_KEY_Q)) {
      break;
    }
    if (input.getKeyState(GLFW_KEY_SPACE)) {
      simulating = !simulating;
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot");
  test1();
}
