//
// Created by sim on 9/27/19.
//

#include "actor_builder.h"
#include "articulation_builder.h"
#include "joint_pub_node.h"
#include "joint_trajectory_controller.h"
#include "optifuser_renderer.h"
#include "simulation.h"
#include "urdf/urdf_loader.h"
#include "velocity_control_service.h"
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
  PxTransform boxPose({0.5f + boxLength, 0.f, -0.5f + boxLength + 1.f}, PxIdentity);
  sim.addGround(0.0);

  auto loader = URDF::URDFLoader(sim);
  loader.load("../assets/179/test.urdf")
      ->articulation->teleportRootLink({{1.2, 0, 0.8}, PxIdentity}, true);

  auto wrapper =
      loader.loadKinematic("../assets/robot/all_robot.urdf");
  auto timestep = sim.getTimestep();

  // ROS
  auto nh = std::make_shared<ros::NodeHandle>();

  robot_interface::JointPubNode node(wrapper, 60, 1000, "/joint_states", nh);

  robot_interface::GroupControllerNode controller(wrapper, "right_arm", timestep, nh);

  //  std::vector<std::string> serviceJoints = {
  //      "right_gripper_finger1_joint", "right_gripper_finger2_joint",
  //      "right_gripper_finger3_joint"};
  //  robot_interface::VelocityControllerServer service(wrapper, serviceJoints, timestep,
  //  "gripper", nh); wrapper->add_velocity_controller(service.getJointNames(),
  //  service.getQueue()); std::thread th3(&robot_interface::VelocityControllerServer::spin,
  //  &service);

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
  std::vector<PxReal> initQpos(13, 0);
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
