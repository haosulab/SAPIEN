//
// Created by sim on 10/12/19.
//

#pragma once

#include "device/joystick_ps3.h"
namespace sapien::robot {

class MOVOPS3 : public PS3RobotControl {
protected:
  const std::vector<std::string> headJoints = {"pan_joint", "tilt_joint"};
  const std::vector<std::string> bodyJoints = {"linear_joint"};
  float wheel_velocity = 0.8;
  float body_velocity = 0.2;
  float head_velocity = 2;
  JointVelocityController *body;
  JointVelocityController *head;

public:
  inline float get_wheel_velocity() { return wheel_velocity; }
  inline float get_head_velocity() { return head_velocity; }
  inline float get_body_velocity() { return body_velocity; }
  inline void set_wheel_velocity(float v) { wheel_velocity = v; }
  inline void set_body_velocity(float v) { body_velocity = v; }
  inline void set_head_velocity(float v) { head_velocity = v; }

public:
  explicit MOVOPS3(ControllerManger *m) : PS3RobotControl(m) {
    gripper = manger->name2JointVelocityController["right_gripper"].get();
    body = manger->name2JointVelocityController["body"].get();
    head = manger->name2JointVelocityController["head"].get();
    arm_cartesian = manger->name2CartesianVelocityController["right_arm"].get();
    assert(gripper && body && head && arm_cartesian);

    gripperJoints = {"right_gripper_finger1_joint", "right_gripper_finger2_joint",
                     "right_gripper_finger3_joint"};
    gripper_velocity = 3;
    arm_velocity = 0.25;
    arm_angular_velocity = 0.7;

    mode = ControlMode::BODY;
    std::cout << "Using Robot Body Mode." << std::endl;
    arm_cartesian->setAngularVelocity(arm_angular_velocity);
    arm_cartesian->setVelocity(arm_velocity);
    arm_cartesian->toggleJumpTest(true);
    manger->start();
  }
  void step() override {
    PS3RobotControl::step();
    switch (mode) {
    case BODY: {
      if (input->getKey(BUTTON_UP)) {
        pos += PxQuat(angle, {0, 0, 1}).rotate(PxVec3(1, 0, 0)) * time_step * wheel_velocity;
        manger->moveBase({pos, PxQuat(angle, {0, 0, 1})});
      } else if (input->getKey(BUTTON_DOWN)) {
        pos -= PxQuat(angle, {0, 0, 1}).rotate(PxVec3(1, 0, 0)) * time_step * wheel_velocity;
        manger->moveBase({pos, PxQuat(angle, {0, 0, 1})});
      } else if (input->getKey(BUTTON_LEFT)) {
        pos += PxQuat(angle, {0, 0, 1}).rotate(PxVec3(0, 1, 0)) * time_step * wheel_velocity;
        manger->moveBase({pos, PxQuat(angle, {0, 0, 1})});
      } else if (input->getKey(BUTTON_RIGHT)) {
        pos -= PxQuat(angle, {0, 0, 1}).rotate(PxVec3(0, 1, 0)) * time_step * wheel_velocity;
        manger->moveBase({pos, PxQuat(angle, {0, 0, 1})});
      } else if (input->getAxis(AXIS_LEFT_X)) {
        float dir = input->getAxisValue(AXIS_LEFT_X) > 0 ? -1 : 1;
        angle += time_step * wheel_velocity * dir;
        manger->moveBase({pos, PxQuat(angle, {0, 0, 1})});
      } else if (input->getKey(BUTTON_TRIANGLE)) {
        head->moveJoint({"tilt_joint"}, -head_velocity);
      } else if (input->getKey(BUTTON_X)) {
        head->moveJoint({"tilt_joint"}, head_velocity);
      } else if (input->getKey(BUTTON_SQUARE)) {
        head->moveJoint({"pan_joint"}, -head_velocity);
      } else if (input->getKey(BUTTON_CIRCLE)) {
        head->moveJoint({"pan_joint"}, head_velocity);
      } else if (input->getAxis(AXIS_RIGHT_Y)) {
        float dir = input->getAxisValue(AXIS_RIGHT_Y) > 0 ? -1 : 1;
        body->moveJoint(bodyJoints, body_velocity * dir);
      } else {
        activated = false;
      }
      parseGripperControlSignal();
      break;
    }
    case ARM_WORLD: {
      parseArmWorldControlSignal();
      break;
    }
    case ARM_LOCAL: {
      parseArmLocalControlSignal();
      break;
    }
    }
    parseEndingSignal();
  };
};
} // namespace sapien::robot
