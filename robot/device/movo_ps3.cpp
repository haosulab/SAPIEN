//
// Created by sim on 10/12/19.
//
#include "movo_ps3.h"

sapien::robot::MOVOPS3::MOVOPS3(ControllerManger *manger) : MOVO(manger) {
  input = std::make_unique<PS3>();
}
void sapien::robot::MOVOPS3::step() {
  if (input->getKey(BUTTON_SELECT)) {
    mode = ControlMode::ARM_WORLD;
  } else if (input->getKey(BUTTON_START)) {
    mode = ControlMode::ARM_LOCAL;
  } else if (input->getKey(BUTTON_PS3)) {
    mode = ControlMode::BODY;
  }
  bool activated = true;
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
      float dir = input->getAxisValue(AXIS_LEFT_X) > 0 ? -1 : 1;
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
      float dir = input->getAxisValue(AXiS_RIGHT_Y) > 0 ? -1 : 1;
      body->moveJoint(bodyJoints, body_velocity * dir);
    } else {
      activated = false;
    }
    break;
  }
  case ARM_WORLD: {
    if (input->getAxis(AXIS_LEFT_X) || input->getAxis(AXiS_LEFT_Y) || input->getKey(BUTTON_UP) ||
        input->getKey(BUTTON_DOWN)) {
      std::array<float, 3> vec = {0, 0, 0};
      vec[1] = -input->getAxisValue(AXIS_LEFT_X);
      vec[0] = -input->getAxisValue(AXiS_LEFT_Y);
      vec[2] = input->getKey(BUTTON_UP) ? 1 : 0;
      vec[2] = input->getKey(BUTTON_DOWN) ? -1 : vec[2];
      right_arm_cartesian->moveRelative(vec, WorldTranslate, continuous);
    } else if (input->getAxis(AXIS_RIGHT_X) || input->getAxis(AXiS_RIGHT_Y) ||
               input->getKey(BUTTON_TRIANGLE) || input->getKey(BUTTON_X)) {
      std::array<float, 3> vec = {0, 0, 0};
      vec[0] = input->getAxisValue(AXIS_RIGHT_X);
      vec[1] = input->getAxisValue(AXiS_RIGHT_Y);
      vec[2] = input->getKey(BUTTON_TRIANGLE) ? 1 : 0;
      vec[2] = input->getKey(BUTTON_X) ? -1 : vec[2];
      right_arm_cartesian->moveRelative(vec, WorldRotate, continuous);
    } else if (input->getKey(BUTTON_L1)) {
      right_gripper->moveJoint(gripperJoints, gripper_velocity);
    } else if (input->getKey(BUTTON_R1)) {
      right_gripper->moveJoint(gripperJoints, -gripper_velocity);
    } else {
      activated = false;
    }
    break;
  }
  case ARM_LOCAL: {
    if (input->getAxis(AXIS_LEFT_X) || input->getAxis(AXiS_LEFT_Y) || input->getKey(BUTTON_UP) ||
        input->getKey(BUTTON_DOWN)) {
      std::array<float, 3> vec = {0, 0, 0};
      vec[1] = -input->getAxisValue(AXIS_LEFT_X);
      vec[2] = -input->getAxisValue(AXiS_LEFT_Y);
      vec[0] = input->getKey(BUTTON_UP) ? 1 : 0;
      vec[0] = input->getKey(BUTTON_DOWN) ? -1 : vec[0];
      right_arm_cartesian->moveRelative(vec, LocalTranslate, continuous);
    } else if (input->getAxis(AXIS_RIGHT_X) || input->getAxis(AXiS_RIGHT_Y) ||
               input->getKey(BUTTON_TRIANGLE) || input->getKey(BUTTON_X)) {
      std::array<float, 3> vec = {0, 0, 0};
      vec[1] = input->getAxisValue(AXIS_RIGHT_X);
      vec[2] = input->getAxisValue(AXiS_RIGHT_Y);
      vec[0] = input->getKey(BUTTON_TRIANGLE) ? 1 : 0;
      vec[0] = input->getKey(BUTTON_X) ? -1 : vec[0];
      right_arm_cartesian->moveRelative(vec, LocalRotate, continuous);
    } else if (input->getKey(BUTTON_L1)) {
      right_gripper->moveJoint(gripperJoints, gripper_velocity);
    } else if (input->getKey(BUTTON_R1)) {
      right_gripper->moveJoint(gripperJoints, -gripper_velocity);
    } else {
      activated = false;
    }
    break;
  }
  }
  continuous = activated;
}
sapien::robot::MOVOPS3::~MOVOPS3() { input->shutdown(); }
