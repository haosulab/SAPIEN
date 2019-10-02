//
// Created by sim on 9/27/19.
//

#include "actor_builder.h"
#include "articulation_builder.h"
#include "group_controller_node.h"
#include "joint_pub_node.h"
#include "optifuser_renderer.h"
#include "simulation.h"
#include "urdf/urdf_loader.h"
#include <extensions/PxDefaultCpuDispatcher.h>
#include <extensions/PxSimpleFactory.h>
#include <foundation/PxMat33.h>
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
  sim.addGround(-0.3f);

  auto loader = URDF::URDFLoader(sim);
  std::unique_ptr<PxKinematicsArticulationWrapper> unique_wrapper =
      loader.loadKinematic("../assets/robot/arm_without_gazebo.urdf");
  auto wrapper = unique_wrapper.get();
  sim.mKinematicArticulationWrappers.push_back(std::move(unique_wrapper));
  auto timestep = sim.getTimestep();

  // ROS
  auto nh = std::make_shared<ros::NodeHandle>();
  robot_interface::JointPubNode node(wrapper->get_queue(), wrapper->get_drive_joint_name(), 30,
                                     1000, "/joint_states", nh);
  std::thread th1(&robot_interface::JointPubNode::spin, &node);

  robot_interface::GroupControllerNode controller(wrapper->get_drive_joint_name(), "right_arm",
                                                  "physx", timestep, nh);
  wrapper->addJointController(wrapper->get_drive_joint_name(), controller.getQueue());
  std::thread th2(&robot_interface::GroupControllerNode::spin, &controller);

  //  auto queue = controller.getQueue();
  //  std::vector<PxReal> initQpos = {-1.93475823254,
  //                                  -1.53188487338,
  //                                  0.951243371548,
  //                                  -2.24179359416,
  //                                  0.344180286477,
  //                                  0.649430580507,
  //                                  -1.41300076449,
  //                                  0,
  //                                  0,
  //                                  0};
  std::vector<PxReal> initQpos = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  wrapper->set_qpos(initQpos);
  //    wrapper->set_drive_target(initQpos);
  //  wrapper->set_qvel({0.1, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0, 0, 0});
  //  wrapper->set_qvel({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  bool simulating = true;
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
      usleep(1000);
      //      queue->pushValue(std::vector<float> {1,1,1,1,1,1,1,1,1,1});
    }
    renderer.render();
    auto input = Optifuser::getInput();
    if (input.getKeyState(GLFW_KEY_Q)) {
      break;
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot");
  test1();
}
