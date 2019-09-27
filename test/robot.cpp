//
// Created by sim on 9/25/19.
//

#include "actor_builder.h"
#include "optifuser_renderer.h"
#include "robot/joint_pub_node.h"
#include "robot/physx_robot_interface.h"
#include "urdf/urdf_loader.h"
#include <ros/ros.h>
#include <thread>

void test1() {
  OptifuserRenderer renderer;
  renderer.init();
  renderer.cam.position = {0.5, -4, 0.5};
  renderer.cam.forward = {0, 1, 0};
  renderer.cam.up = {0, 0, 1};

  PxSimulation sim;
  sim.setRenderer(&renderer);
  sim.setTimestep(1.f / 60.f);

  auto builder = sim.createActorBuilder();
  PxReal boxLength = 1.0f;
  // PxVec3 boxSize = {0.3, 0.3, 0.3};
  PxVec3 boxSize = {boxLength, boxLength, boxLength};
  PxTransform boxPose({0.5f + boxLength, 0.f, -0.5f + boxLength}, PxIdentity);
  builder->addBoxShape(boxPose, boxSize, nullptr, 1.f);
  builder->addBoxVisual(boxPose, boxSize);
  auto box = builder->build();
  // setFilterData(box, {0, 0, PxPairFlag::eMODIFY_CONTACTS, 0});

  sim.addGround(-0.3f);

  auto loader = URDFLoader(sim);
  auto articulation = loader.loadKinematic("../assets/robot/arm_without_gazabo.urdf");
  // articulation->getDofs()
  articulation.setAllPos({-1.93475823254, -1.53188487338, 0.951243371548, -2.24179359416,
                          0.344180286477, 0.649430580507, -1.41300076449});
  articulation.setAllVelocity({0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
  auto root = articulation.getRoot();
  std::vector<KJoint *> stack;
  stack.push_back(root);
  bool simulating = false;
  sim.step();
  sim.updateRenderer();
  articulation.update();

  robot_interface::PxRobotInterface interface = robot_interface::PxRobotInterface(sim);
  auto queue = robot_interface::ThreadSafeQueue();
  interface.addKinematicsArticulation("arm_without_gazebo", &articulation);
  auto nh = std::make_shared<ros::NodeHandle>();
  auto pub = robot_interface::JointStatePublisher(&interface, 100, "/joint_states", "arm_without_gazebo", nh);
  std::thread tPub(&robot_interface::JointStatePublisher::publishLoop, &pub, std::ref(queue));

  while (true) {
    if (simulating) {
      articulation.update();
      PxTransform pose = root->children[0]->childLink->getGlobalPose();
      sim.step();
      sim.updateRenderer();
      queue.push(interface.getJointAngle("arm_without_gazebo"));
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

int main(int argc, char **argv) { test1(); }
