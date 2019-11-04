//
// Created by sim on 10/31/19.
//

#include "xarm6.h"
sapien::robot::XArm6::XArm6(sapien::robot::ControllerManger *manger) {
  manger->createJointPubNode(100, 900);
  gripper = manger->createJointVelocityController(gripperJoints, "gripper");
  arm_cartesian = manger->createCartesianVelocityController("xarm6");
  timestep = manger->time_step;

  arm_cartesian->setAngularVelocity(arm_cartesian_angular_velocity);
  arm_cartesian->setVelocity(arm_cartesian_velocity);
  arm_cartesian->toggleJumpTest(true);

  manger->start();
}
void sapien::robot::XArm6::set_gripper_velocity(float v) { gripper_velocity = v; }
