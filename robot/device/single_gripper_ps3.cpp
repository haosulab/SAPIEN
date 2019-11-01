//
// Created by sim on 10/31/19.
//

#include "single_gripper_ps3.h"
sapien::robot::KinovaGripperPS3::KinovaGripperPS3(sapien::robot::ControllerManger *manger)
    : SingleKinovaGripper(manger) {
  input = std::make_unique<PS3>();
}
sapien::robot::KinovaGripperPS3::~KinovaGripperPS3() { input->shutdown(); }
std::vector<int> sapien::robot::KinovaGripperPS3::get_cache() {
  auto dest = input->exportButtonStates();
  auto src = input->exportAxisStates();
  dest.insert(dest.end(), std::make_move_iterator(src.begin()),
              std::make_move_iterator(src.end()));
  return dest;
}
void sapien::robot::KinovaGripperPS3::set_cache(const std::vector<int> &cache) {
  input->setCache(std::vector<int>(cache.begin(), cache.begin() + PS3_BUTTON_COUNT),
                  std::vector<int>(cache.begin() + PS3_BUTTON_COUNT, cache.end()));
}
void sapien::robot::KinovaGripperPS3::step() {
  switch (input->mode) {
  default:
    input->saveCache();
    break;
  case REPLAY:
    break;
  }

  bool activated = true;
  if (input->getKey(BUTTON_UP)) {
    translation->moveJoint({"z_axis_joint"}, translation_velocity);
  } else if (input->getKey(BUTTON_DOWN)) {
    translation->moveJoint({"z_axis_joint"}, -translation_velocity);
  } else if (input->getAxis(AXIS_LEFT_X) || input->getAxis(AXIS_LEFT_Y)) {
    float dir_x = input->getAxisValue(AXIS_LEFT_X) > 0 ? -1 : 1;
    translation->moveJoint({"y_axis_joint"}, translation_velocity * dir_x);
    float dir_y = input->getAxisValue(AXIS_LEFT_Y) > 0 ? -1 : 1;
    translation->moveJoint({"x_axis_joint"}, translation_velocity * dir_y);
  } else if (input->getKey(BUTTON_L1)) {
    gripper->moveJoint(gripperJoints, gripper_velocity);
  } else if (input->getKey(BUTTON_R1)) {
    grasped = false;
    gripper->moveJoint(gripperJoints, -gripper_velocity);
  } else if (input->getKey(BUTTON_TRIANGLE)) {
    rotation->moveJoint({"r_rotation_joint"}, rotation_velocity);
  } else if (input->getKey(BUTTON_X)) {
    rotation->moveJoint({"r_rotation_joint"}, -rotation_velocity);
  } else if (input->getAxis(AXiS_RIGHT_Y) || input->getAxis(AXIS_RIGHT_X)) {
    float dir_y = input->getAxisValue(AXiS_RIGHT_Y) > 0 ? -1 : 1;
    float dir_x = input->getAxisValue(AXIS_RIGHT_X) > 0 ? -1 : 1;
    rotation->moveJoint({"p_rotation_joint"}, rotation_velocity * dir_x);
    rotation->moveJoint({"y_rotation_joint"}, rotation_velocity * dir_y);
  } else if (input->getAxis(AXIS_LEFT_L2)) {
    grasped = true;
  } else {
    activated = false;
  }
  if(grasped){
    gripper->moveJoint(gripperJoints, gripper_velocity);
  }
  if (input->getKey(BUTTON_LEFT) && input->getKey(BUTTON_CIRCLE)) {
    throw std::runtime_error("PS3 user interrupt.");
  }
  continuous = activated;
}
