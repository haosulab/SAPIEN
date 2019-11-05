//
// Created by sim on 10/31/19.
//

#pragma once

#include "device/joystick_ps3.h"
namespace sapien::robot {

class KinovaGripperPS3 : public PS3RobotControl {

public:
  explicit KinovaGripperPS3(ControllerManger *manger) : PS3RobotControl(manger) {
    gripper = manger->name2JointVelocityController["right_gripper"].get();
    translation = manger->name2JointVelocityController["root_translation"].get();
    rotation = manger->name2JointVelocityController["root_rotation"].get();

    gripperJoints = {"right_gripper_finger1_joint", "right_gripper_finger2_joint",
                     "right_gripper_finger3_joint"};
    gripper_velocity = 1;
    translation_velocity = 0.15;
    rotation_velocity = 0.5;
    std::cout << "Mode toggle function will not make influence for Single Gripper." << std::endl;
  };
  void step() override {
    PS3RobotControl::step();
    PS3RobotControl::parseRootJointControlSignal();
    PS3RobotControl::parseGripperControlSignal();
    PS3RobotControl::parseEndingSignal();
  };
};
} // namespace sapien::robot
