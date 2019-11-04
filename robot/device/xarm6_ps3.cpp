//
// Created by sim on 10/12/19.
//
#include "xarm6_ps3.h"

sapien::robot::XArm6PS3::XArm6PS3(ControllerManger *manger) : XArm6(manger) {
  input = std::make_unique<PS3>();
}
void sapien::robot::XArm6PS3::step() {
  switch (input->mode) {
  default:
    input->saveCache();
    break;
  case REPLAY:
    break;
  }

  bool activated = true;
  if (input->getAxis(AXIS_LEFT_X) || input->getAxis(AXIS_LEFT_Y) || input->getKey(BUTTON_UP) ||
      input->getKey(BUTTON_DOWN)) {
    std::array<float, 3> vec = {0, 0, 0};
    vec[1] = -input->getAxisValue(AXIS_LEFT_X);
    vec[0] = -input->getAxisValue(AXIS_LEFT_Y);
    vec[2] = input->getKey(BUTTON_UP) ? 1 : 0;
    vec[2] = input->getKey(BUTTON_DOWN) ? -1 : vec[2];
    arm_cartesian->moveRelative(vec, WorldTranslate, continuous);
  } else if (input->getAxis(AXIS_RIGHT_X) || input->getAxis(AXIS_RIGHT_Y) ||
             input->getKey(BUTTON_TRIANGLE) || input->getKey(BUTTON_X)) {
    std::array<float, 3> vec = {0, 0, 0};
    vec[0] = input->getAxisValue(AXIS_RIGHT_X);
    vec[1] = input->getAxisValue(AXIS_RIGHT_Y);
    vec[2] = input->getKey(BUTTON_TRIANGLE) ? 1 : 0;
    vec[2] = input->getKey(BUTTON_X) ? -1 : vec[2];
    arm_cartesian->moveRelative(vec, WorldRotate, continuous);
  } else if (input->getKey(BUTTON_L1)) {
    gripper->moveJoint(gripperJoints, gripper_velocity);
  } else if (input->getKey(BUTTON_R1)) {
    gripper->moveJoint(gripperJoints, -gripper_velocity);
  } else {
    activated = false;
  }
  if (input->getKey(BUTTON_LEFT) && input->getKey(BUTTON_CIRCLE)) {
    throw std::runtime_error("PS3 user interrupt.");
  }
  continuous = activated;
}
sapien::robot::XArm6PS3::~XArm6PS3() { input->shutdown(); }
std::vector<int> sapien::robot::XArm6PS3::get_cache() {
  auto dest = input->exportButtonStates();
  auto src = input->exportAxisStates();
  dest.insert(dest.end(), std::make_move_iterator(src.begin()),
              std::make_move_iterator(src.end()));
  return dest;
}
void sapien::robot::XArm6PS3::set_cache(const std::vector<int> &cache) {
  input->setCache(std::vector<int>(cache.begin(), cache.begin() + PS3_BUTTON_COUNT),
                  std::vector<int>(cache.begin() + PS3_BUTTON_COUNT, cache.end()));
}
