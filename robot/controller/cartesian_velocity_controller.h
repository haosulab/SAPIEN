//
// Created by sim on 10/3/19.
//
#pragma once
#include "controllable_articulation_wrapper.h"
#include <memory>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>

namespace sapien::robot {

enum CartesianCommand {
  X_F = 0,
  Y_F = 1,
  Z_F = 2,
  ROLL_F = 3,
  PITCH_F = 4,
  YAW_F = 5,
  X_B = 6,
  Y_B = 7,
  Z_B = 8,
  ROLL_B = 9,
  PITCH_B = 10,
  YAW_B = 11
};

enum MoveType { WorldTranslate, WorldRotate, LocalTranslate, LocalRotate };

class CartesianVelocityController {
private:
  robot_model_loader::RobotModelLoader loader;
  robot_model::RobotModelPtr kinematicModel;
  std::unique_ptr<robot_state::RobotState> state;
  const robot_state::JointModelGroup *jointModelGroup;
  ros::Subscriber sub;
  ros::NodeHandle *mNodeHandle;

  // Cache
  std::vector<std::string> jointName;
  Eigen::Isometry3d currentPose;
  std::string eeName;
  std::string jointStateTopicName;
  std::vector<uint32_t> jointIndex2Topic = {};
  std::vector<double> jointValue;
  std::vector<float> jointValueFloat;

  // Communication
  std::unique_ptr<ThreadSafeQueue> mQueue;
  bool jointJumpCheck = false;
  float transStepSize;
  float rotStepSize;
  std::vector<Eigen::Isometry3d> cartesianMatrix =
      std::vector<Eigen::Isometry3d>(12, Eigen::Isometry3d::Identity());
  static bool testJointSpaceJump(const std::vector<double> &q1, const std::vector<double> &q2,
                                 double threshold);

public:
  float timestep;

private:
  void updateCurrentPose();
  void buildCartesianVelocityCache();
  void buildCartesianAngularVelocityCache();

public:
  CartesianVelocityController(ControllableArticulationWrapper *wrapper,
                              const std::string &groupName, float timestep, ros::NodeHandle *nh,
                              const std::string &robotName = "");
  void moveRelative(CartesianCommand type, bool continuous = false);
  void moveRelative(const std::array<float, 3> &T, MoveType type, bool continuous);
  float getVelocity() const;
  void setVelocity(float v);
  float getAngularVelocity() const;
  void setAngularVelocity(float v);
  void toggleJumpTest(bool enable);

  // Used for input device
};
} // namespace sapien::robot
