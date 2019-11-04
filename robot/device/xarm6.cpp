//
// Created by sim on 10/31/19.
//

#include "xarm6.h"
sapien::robot::XArm6::XArm6(sapien::robot::ControllerManger *manger) {
  manger->createJointPubNode(100, 500);
  gripper = manger->createJointVelocityController(gripperJoints, "gripper");
  arm_cartesian = manger->createCartesianVelocityController("xarm6");
  timestep = manger->time_step;

  manger->start();
}
void sapien::robot::XArm6::set_gripper_velocity(float v) { gripper_velocity = v; }
