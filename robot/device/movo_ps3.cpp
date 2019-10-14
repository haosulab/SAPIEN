//
// Created by sim on 10/12/19.
//
#include "movo_ps3.h"

sapien::robot::MOVOPS3::MOVOPS3(ControllerManger *manger) : manger(manger) {
  input = std::make_unique<PS3>();
  right_gripper = manger->createJointVelocityController(gripperJoints, "right_gripper");
  body = manger->createJointVelocityController(bodyJoints, "body");
  head = manger->createJointVelocityController(headJoints, "head");
  right_arm_cartesian = manger->createCartesianVelocityController("right_arm");
  timestep = manger->timestep;
  manger->start();
}
void sapien::robot::MOVOPS3::set_arm_velocity(float v) { right_arm_cartesian->setVelocity(v); }
void sapien::robot::MOVOPS3::set_arm_angular_velocity(float v) {
  right_arm_cartesian->setVelocity(v);
}
void sapien::robot::MOVOPS3::step() {
  if (input->getKey(BUTTON_SELECT)) {
    mode = ControlMode::ARM_WORLD;
  } else if (input->getKey(BUTTON_START)) {
    mode = ControlMode::ARM_LOCAL;
  } else if (input->getKey(BUTTON_PS3)) {
    mode = ControlMode::BODY;
  }
  switch (mode) {
  case BODY: {
    if (input->getKey(BUTTON_UP)) {
      pos += PxQuat(angle, {0, 0, 1}).rotate(PxVec3(1, 0, 0)) * timestep * wheel_velocity;
      manger->movoBase({pos, PxQuat(angle, {0, 0, 1})});
    } else if (input->getKey(BUTTON_DOWN)) {
      pos -= PxQuat(angle, {0, 0, 1}).rotate(PxVec3(1, 0, 0)) * timestep * wheel_velocity;
      manger->movoBase({pos, PxQuat(angle, {0, 0, 1})});
    } else if (input->getKey(BUTTON_LEFT)) {
      pos += PxQuat(angle, {0, 0, 1}).rotate(PxVec3(0, 1, 0)) * timestep * wheel_velocity;
      manger->movoBase({pos, PxQuat(angle, {0, 0, 1})});
    } else if (input->getKey(BUTTON_RIGHT)) {
      pos -= PxQuat(angle, {0, 0, 1}).rotate(PxVec3(0, 1, 0)) * timestep * wheel_velocity;
      manger->movoBase({pos, PxQuat(angle, {0, 0, 1})});
    } else if (input->getAxis(AXIS_LEFT_X)) {
      float dir = input->getAxis(AXIS_LEFT_X) > 0 ? -1 : 1;
      angle += timestep * wheel_velocity * dir;
      manger->movoBase({pos, PxQuat(angle, {0, 0, 1})});
    } else if (input->getKey(BUTTON_L1)) {
      right_gripper->moveJoint(gripperJoints, gripper_velocity);
    } else if (input->getKey(BUTTON_R1)) {
      right_gripper->moveJoint(gripperJoints, -gripper_velocity);
    } else if (input->getKey(BUTTON_TRIANGLE)) {
      head->moveJoint({"tilt_joint"}, -head_velocity);
    } else if (input->getKey(BUTTON_X)) {
      head->moveJoint({"tilt_joint"}, head_velocity);
    } else if (input->getKey(BUTTON_SQUARE)) {
      head->moveJoint({"pan_joint"}, -head_velocity);
    } else if (input->getKey(BUTTON_CIRCLE)) {
      head->moveJoint({"pan_joint"}, head_velocity);
    } else if (input->getAxis(AXiS_RIGHT_Y)) {
      float dir = input->getAxis(AXiS_RIGHT_Y) > 0 ? -1 : 1;
      body->moveJoint(bodyJoints, body_velocity * dir);
    }
  }
  }
}
void sapien::robot::MOVOPS3::set_wheel_velocity(float v) { wheel_velocity = v; }
void sapien::robot::MOVOPS3::close() { input->shutdown(); }
