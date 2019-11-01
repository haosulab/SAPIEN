//
// Created by sim on 10/31/19.
//

#include "single_gripper.h"
sapien::robot::SingleKinovaGripper::SingleKinovaGripper(sapien::robot::ControllerManger *manger) {
  manger->createJointPubNode(100, 500);
  gripper = manger->createJointVelocityController(gripperJoints, "right_gripper");
  translation = manger->createJointVelocityController(translationJoints, "gripper_translation");
  rotation = manger->createJointVelocityController(rotationJoints, "gripper_rotation");
  timestep = manger->time_step;

  manger->start();
}
void sapien::robot::SingleKinovaGripper::set_gripper_velocity(float v) { gripper_velocity = v; }
void sapien::robot::SingleKinovaGripper::set_translation_velocity(float v) {
  translation_velocity = v;
}
void sapien::robot::SingleKinovaGripper::set_rotation_velocity(float v) { rotation_velocity = v; }
