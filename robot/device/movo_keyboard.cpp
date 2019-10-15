//
// Created by sim on 10/14/19.
//
#include "movo_keyboard.h"

#define AXIS_LEFT_X_POSITIVE GLFW_KEY_LEFT
#define AXIS_LEFT_Y_POSITIVE GLFW_KEY_UP
#define AXIS_LEFT_X_NEGATIVE GLFW_KEY_RIGHT
#define AXIS_LEFT_Y_NEGATIVE GLFW_KEY_DOWN
#define AXIS_RIGHT_X_POSITIVE GLFW_KEY_L
#define AXIS_RIGHT_Y_POSITIVE GLFW_KEY_I
#define AXIS_RIGHT_X_NEGATIVE GLFW_KEY_J
#define AXIS_RIGHT_Y_NEGATIVE GLFW_KEY_K
#define BUTTON_L1 GLFW_KEY_U
#define BUTTON_R1 GLFW_KEY_O
#define BUTTON_UP GLFW_KEY_T
#define BUTTON_DOWN GLFW_KEY_G
#define BUTTON_LEFT GLFW_KEY_F
#define BUTTON_RIGHT GLFW_KEY_H
#define BUTTON_CIRCLE GLFW_KEY_V
#define BUTTON_SQUARE GLFW_KEY_B
#define BUTTON_TRIANGLE GLFW_KEY_N
#define BUTTON_X GLFW_KEY_M
#define BUTTON_SELECT GLFW_KEY_2
#define BUTTON_START GLFW_KEY_3
#define BUTTON_PS3 GLFW_KEY_1

sapien::robot::MOVOKeyboard::MOVOKeyboard(ControllerManger *manger) : MOVO(manger) {}
void sapien::robot::MOVOKeyboard::step() {
  auto gl_input = Optifuser::getInput();
  if (gl_input.getKeyState(BUTTON_SELECT)) {
    mode = ControlMode::ARM_WORLD;
  } else if (gl_input.getKeyState(BUTTON_START)) {
    mode = ControlMode::ARM_LOCAL;
  } else if (gl_input.getKeyState(BUTTON_PS3)) {
    mode = ControlMode::BODY;
  }
  bool activated = true;
  switch (mode) {
  case BODY: {
    if (gl_input.getKeyState(BUTTON_UP)) {
      pos += PxQuat(angle, {0, 0, 1}).rotate(PxVec3(1, 0, 0)) * timestep * wheel_velocity;
      manger->movoBase({pos, PxQuat(angle, {0, 0, 1})});
    } else if (gl_input.getKeyState(BUTTON_DOWN)) {
      pos -= PxQuat(angle, {0, 0, 1}).rotate(PxVec3(1, 0, 0)) * timestep * wheel_velocity;
      manger->movoBase({pos, PxQuat(angle, {0, 0, 1})});
    } else if (gl_input.getKeyState(BUTTON_LEFT)) {
      pos += PxQuat(angle, {0, 0, 1}).rotate(PxVec3(0, 1, 0)) * timestep * wheel_velocity;
      manger->movoBase({pos, PxQuat(angle, {0, 0, 1})});
    } else if (gl_input.getKeyState(BUTTON_RIGHT)) {
      pos -= PxQuat(angle, {0, 0, 1}).rotate(PxVec3(0, 1, 0)) * timestep * wheel_velocity;
      manger->movoBase({pos, PxQuat(angle, {0, 0, 1})});
    } else if (gl_input.getKeyState(AXIS_LEFT_X_POSITIVE) ||
               gl_input.getKeyState(AXIS_LEFT_X_NEGATIVE)) {
      float dir = gl_input.getKeyState(AXIS_LEFT_X_POSITIVE) ? 1 : 0;
      dir = gl_input.getKeyState(AXIS_LEFT_X_NEGATIVE) ? -1 : dir;
      angle += timestep * wheel_velocity * dir;
      manger->movoBase({pos, PxQuat(angle, {0, 0, 1})});
    } else if (gl_input.getKeyState(BUTTON_L1)) {
      right_gripper->moveJoint(gripperJoints, gripper_velocity);
    } else if (gl_input.getKeyState(BUTTON_R1)) {
      right_gripper->moveJoint(gripperJoints, -gripper_velocity);
    } else if (gl_input.getKeyState(BUTTON_TRIANGLE)) {
      head->moveJoint({"tilt_joint"}, -head_velocity);
    } else if (gl_input.getKeyState(BUTTON_X)) {
      head->moveJoint({"tilt_joint"}, head_velocity);
    } else if (gl_input.getKeyState(BUTTON_SQUARE)) {
      head->moveJoint({"pan_joint"}, -head_velocity);
    } else if (gl_input.getKeyState(BUTTON_CIRCLE)) {
      head->moveJoint({"pan_joint"}, head_velocity);
    } else if (gl_input.getKeyState(AXIS_RIGHT_Y_POSITIVE) ||
               gl_input.getKeyState(AXIS_RIGHT_Y_NEGATIVE)) {
      float dir = gl_input.getKeyState(AXIS_RIGHT_Y_POSITIVE) ? 1 : 0;
      dir = gl_input.getKeyState(AXIS_RIGHT_Y_NEGATIVE) ? -1 : dir;
      body->moveJoint(bodyJoints, body_velocity * dir);
    } else {
      activated = false;
    }
    break;
  }
  case ARM_WORLD: {
    if (gl_input.getKeyState(AXIS_LEFT_X_POSITIVE) || gl_input.getKeyState(AXIS_LEFT_X_NEGATIVE) ||
        gl_input.getKeyState(AXIS_LEFT_Y_POSITIVE) || gl_input.getKeyState(AXIS_LEFT_Y_NEGATIVE) ||
        gl_input.getKeyState(BUTTON_UP) || gl_input.getKeyState(BUTTON_DOWN)) {
      std::array<float, 3> vec = {0, 0, 0};
      vec[1] = gl_input.getKeyState(AXIS_LEFT_X_POSITIVE) ? 1 : 0;
      vec[1] = gl_input.getKeyState(AXIS_LEFT_X_NEGATIVE) ? -1 : vec[1];
      vec[0] = gl_input.getKeyState(AXIS_LEFT_Y_POSITIVE) ? 1 : 0;
      vec[0] = gl_input.getKeyState(AXIS_LEFT_Y_NEGATIVE) ? -1 : vec[0];
      vec[2] = gl_input.getKeyState(BUTTON_UP) ? 1 : 0;
      vec[2] = gl_input.getKeyState(BUTTON_DOWN) ? -1 : vec[2];
      right_arm_cartesian->moveRelative(vec, WorldTranslate, continuous);
    } else if (gl_input.getKeyState(AXIS_RIGHT_X_POSITIVE) ||
               gl_input.getKeyState(AXIS_RIGHT_Y_POSITIVE) ||
               gl_input.getKeyState(AXIS_RIGHT_X_NEGATIVE) ||
               gl_input.getKeyState(AXIS_RIGHT_Y_NEGATIVE) ||
               gl_input.getKeyState(BUTTON_TRIANGLE) || gl_input.getKeyState(BUTTON_X)) {
      std::array<float, 3> vec = {0, 0, 0};
      vec[0] = gl_input.getKeyState(AXIS_RIGHT_X_POSITIVE) ? 1 : 0;
      vec[0] = gl_input.getKeyState(AXIS_RIGHT_X_NEGATIVE) ? -1 : vec[0];
      vec[1] = gl_input.getKeyState(AXIS_RIGHT_Y_POSITIVE) ? 1 : 0;
      vec[1] = gl_input.getKeyState(AXIS_RIGHT_Y_NEGATIVE) ? -1 : vec[1];
      vec[2] = gl_input.getKeyState(BUTTON_TRIANGLE) ? 1 : 0;
      vec[2] = gl_input.getKeyState(BUTTON_X) ? -1 : vec[2];
      right_arm_cartesian->moveRelative(vec, WorldRotate, continuous);
    } else if (gl_input.getKeyState(BUTTON_L1)) {
      right_gripper->moveJoint(gripperJoints, gripper_velocity);
    } else if (gl_input.getKeyState(BUTTON_R1)) {
      right_gripper->moveJoint(gripperJoints, -gripper_velocity);
    } else {
      activated = false;
    }
    break;
  }
  case ARM_LOCAL: {
    if (gl_input.getKeyState(AXIS_LEFT_X_POSITIVE) || gl_input.getKeyState(AXIS_LEFT_X_NEGATIVE) ||
        gl_input.getKeyState(AXIS_LEFT_Y_POSITIVE) || gl_input.getKeyState(AXIS_LEFT_Y_NEGATIVE) ||
        gl_input.getKeyState(BUTTON_UP) || gl_input.getKeyState(BUTTON_DOWN)) {
      std::array<float, 3> vec = {0, 0, 0};
      vec[1] = gl_input.getKeyState(AXIS_LEFT_X_POSITIVE) ? 1 : 0;
      vec[1] = gl_input.getKeyState(AXIS_LEFT_X_NEGATIVE) ? -1 : vec[1];
      vec[0] = gl_input.getKeyState(AXIS_LEFT_Y_POSITIVE) ? 1 : 0;
      vec[0] = gl_input.getKeyState(AXIS_LEFT_Y_NEGATIVE) ? -1 : vec[0];
      vec[2] = gl_input.getKeyState(BUTTON_UP) ? 1 : 0;
      vec[2] = gl_input.getKeyState(BUTTON_DOWN) ? -1 : vec[2];
      right_arm_cartesian->moveRelative(vec, LocalTranslate, continuous);
    } else if (gl_input.getKeyState(AXIS_RIGHT_X_POSITIVE) ||
               gl_input.getKeyState(AXIS_RIGHT_Y_POSITIVE) ||
               gl_input.getKeyState(AXIS_RIGHT_X_NEGATIVE) ||
               gl_input.getKeyState(AXIS_RIGHT_Y_NEGATIVE) ||
               gl_input.getKeyState(BUTTON_TRIANGLE) || gl_input.getKeyState(BUTTON_X)) {
      std::array<float, 3> vec = {0, 0, 0};
      vec[0] = gl_input.getKeyState(AXIS_RIGHT_X_POSITIVE) ? 1 : 0;
      vec[0] = gl_input.getKeyState(AXIS_RIGHT_X_NEGATIVE) ? -1 : vec[0];
      vec[1] = gl_input.getKeyState(AXIS_RIGHT_Y_POSITIVE) ? 1 : 0;
      vec[1] = gl_input.getKeyState(AXIS_RIGHT_Y_NEGATIVE) ? -1 : vec[1];
      vec[2] = gl_input.getKeyState(BUTTON_TRIANGLE) ? 1 : 0;
      vec[2] = gl_input.getKeyState(BUTTON_X) ? -1 : vec[2];
      right_arm_cartesian->moveRelative(vec, LocalRotate, continuous);
    } else if (gl_input.getKeyState(BUTTON_L1)) {
      right_gripper->moveJoint(gripperJoints, gripper_velocity);
    } else if (gl_input.getKeyState(BUTTON_R1)) {
      right_gripper->moveJoint(gripperJoints, -gripper_velocity);
    } else {
      activated = false;
    }
    break;
  }
  }
  continuous = activated;
}
