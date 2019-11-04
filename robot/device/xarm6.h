//
// Created by sim on 10/31/19.
//
#pragma once

#include "controller/controller_manger.h"
#include <memory>
namespace sapien::robot {

class XArm6 {
protected:
  ControllerManger *manger;
  bool reset = true;
  bool continuous = true;

  const std::vector<std::string> gripperJoints = {"drive_joint",
                                                  "left_finger_joint",
                                                  "left_inner_knuckle_joint",
                                                  "right_outer_knuckle_joint",
                                                  "right_finger_joint",
                                                  "right_inner_knuckle_joint"};

  bool grasped;
  float timestep;

public:
  JointVelocityController *gripper;
  CartesianVelocityController *arm_cartesian;

  float gripper_velocity = 1;

  bool startRecord = false;

public:
  explicit XArm6(ControllerManger *manger);
  virtual ~XArm6() = default;
  void set_gripper_velocity(float v);
};
} // namespace sapien::robot
