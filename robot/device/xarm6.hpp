//
// Created by sim on 10/31/19.
//
#pragma once

#include "device/joystick_ps3.h"
#include <memory>
namespace sapien::robot {
class XArm6PS3 : public PS3RobotControl {

public:
  explicit XArm6PS3(ControllerManger *manger) : PS3RobotControl(manger) {
    gripper = manger->name2JointVelocityController["gripper"].get();
    arm_cartesian = manger->name2CartesianVelocityController["xarm6"].get();

    gripperJoints = {"drive_joint",
                     "left_finger_joint",
                     "left_inner_knuckle_joint",
                     "right_outer_knuckle_joint",
                     "right_finger_joint",
                     "right_inner_knuckle_joint"};
    gripper_velocity = 1;
    arm_velocity = 0.10;
    arm_angular_velocity = 0.3;

    arm_cartesian->setAngularVelocity(arm_angular_velocity);
    arm_cartesian->setVelocity(arm_velocity);
    arm_cartesian->toggleJumpTest(true);
    manger->start();
  };
  void step() override {
    PS3RobotControl::step();
    switch (mode) {
    case BODY: {
      std::cerr << "Body mode is not supported for XArm" << std::endl;
      break;
    }
    case ARM_WORLD: {
      PS3RobotControl::parseArmWorldControlSignal();
      break;
    }
    case ARM_LOCAL: {
      PS3RobotControl::parseArmLocalControlSignal();
      break;
    }
    }
    PS3RobotControl::parseEndingSignal();
  };
};

} // namespace sapien::robot
