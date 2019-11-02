//
// Created by sim on 10/31/19.
//
#pragma once

#include "controller/controller_manger.h"
#include <memory>
namespace sapien::robot {

class SingleKinovaGripper {
protected:
  ControllerManger *manger;
  bool reset = true;
  bool continuous = true;

  const std::vector<std::string> gripperJoints = {
      "right_gripper_finger1_joint", "right_gripper_finger2_joint", "right_gripper_finger3_joint"};

  const std::vector<std::string> translationJoints = {"x_axis_joint", "y_axis_joint",
                                                      "z_axis_joint"};
  const std::vector<std::string> rotationJoints = {"r_rotation_joint", "p_rotation_joint",
                                                   "y_rotation_joint"};
  bool grasped;
  float timestep;

public:
  JointVelocityController *gripper;
  JointVelocityController *translation;
  JointVelocityController *rotation;

  float gripper_velocity = 1;
  float translation_velocity = 0.5;
  float rotation_velocity = 1;

  bool startRecord = false;

public:
  explicit SingleKinovaGripper(ControllerManger *manger);
  virtual ~SingleKinovaGripper() = default;
  void set_gripper_velocity(float v);
  void set_translation_velocity(float v);
  void set_rotation_velocity(float v);
};
} // namespace sapien::robot
