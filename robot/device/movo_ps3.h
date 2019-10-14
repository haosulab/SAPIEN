//
// Created by sim on 10/12/19.
//

#pragma once

#include "controller/controller_manger.h"
#include "device/joystick_ps3.h"
#include <memory>
namespace sapien::robot {

enum ControlMode { ARM_WORLD, ARM_LOCAL, BODY };

class MOVOPS3 {
  ControllerManger *manger;
  std::unique_ptr<PS3> input;
  bool reset = true;
  bool continuous = true;
  ControlMode mode = ControlMode::BODY;

  const std::vector<std::string> gripperJoints = {
      "right_gripper_finger1_joint", "right_gripper_finger2_joint", "right_gripper_finger3_joint"};
  const std::vector<std::string> headJoints = {"pan_joint", "tilt_joint"};
  const std::vector<std::string> bodyJoints = {"linear_joint"};

  PxVec3 pos = {0,0,0};
  PxReal angle = 0;
  float timestep;

public:
  CartesianVelocityController *right_arm_cartesian;
  JointVelocityController *right_gripper;
  JointVelocityController *body;
  JointVelocityController *head;

  float gripper_velocity = 3;
  float wheel_velocity = 1;
  float body_velocity = 0.1;
  float head_velocity = 1;
  float arm_cartesian_velocity = 0.2;
  float arm_cartesian_angular_velocity = 0.5;

public:
  explicit MOVOPS3(ControllerManger *manger);
  void set_arm_velocity(float v);
  void set_arm_angular_velocity(float v);
  void set_wheel_velocity(float v);
  void step();
  void close();
};
} // namespace sapien::robot
