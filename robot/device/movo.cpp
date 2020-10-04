//
// Created by sim on 10/14/19.
//

#include "movo.h"
sapien::robot::MOVO::MOVO(ControllerManger *manger) : manger(manger) {
  manger->createJointPubNode(100, 500);
  right_gripper = manger->createJointVelocityController(gripperJoints, "right_gripper");
  body = manger->createJointVelocityController(bodyJoints, "body");
  head = manger->createJointVelocityController(headJoints, "head");
  right_arm_cartesian = manger->createCartesianVelocityController("right_arm");
  timestep = manger->time_step;
  right_arm_cartesian->setVelocity(arm_cartesian_velocity);
  right_arm_cartesian->setAngularVelocity(arm_cartesian_angular_velocity);
  right_arm_cartesian->toggleJumpTest(true);

  manger->start();
}
void sapien::robot::MOVO::set_arm_velocity(float v) {
  right_arm_cartesian->setVelocity(v);
  arm_cartesian_velocity = v;
}
void sapien::robot::MOVO::set_arm_angular_velocity(float v) {
  right_arm_cartesian->setAngularVelocity(v);
  arm_cartesian_angular_velocity = v;
}
void sapien::robot::MOVO::set_wheel_velocity(float v) { wheel_velocity = v; }
