#include "joystick_ps3.h"
#include <cassert>

namespace sapien::ros2 {

PS3::PS3()
    : InputDevice("SHANWAN PS3 GamePad"), buttonCache(PS3_BUTTON_COUNT),
      axisCache(PS3_AXIS_COUNT) {

  for (auto &state : buttonStates) {
    state = 0;
  }
  for (auto &state : axisStates) {
    state = 0;
  }
  if (!shouldStart) {
    std::cerr << "PS3 controller will not start!" << std::endl;
  } else {
    worker = std::thread(&PS3::runThread, this);
    std::cout << "PS3 controller Started" << std::endl;
  }
}
void PS3::runThread() {
  js_event_t event{};
  while (!shouldExit) {
    device.read(reinterpret_cast<char *>(&event), sizeof(js_event_t));
    //    lock.lock();
    if (event.type == 1) {
      assert(event.id >= 0 && event.id < PS3_BUTTON_COUNT);
      buttonStates[event.id] = event.action;
    } else if (event.type == 2) {
      assert(event.id >= 0 && event.id < PS3_AXIS_COUNT);
      axisStates[event.id] = event.action;
    }

    //    lock.unlock();
    usleep(1000);
  }
}
void PS3::shutdown() {
  if (mode != REPLAY) {
    shouldExit = true;
    worker.join();
  }
}
bool PS3::getKey(ButtonId id) { return buttonCache[id]; }
bool PS3::getAxis(AxisId id) { return axisCache[id]; }
float PS3::getAxisValue(AxisId id) { return static_cast<float>(axisCache[id]) / AXIS_CONST; }
void PS3::setMode(PS3Mode option) { mode = option; }
void PS3::saveCache() {
  std::lock_guard<std::mutex> guard(lock);
  std::copy(buttonStates.begin(), buttonStates.end(), buttonCache.begin());
  std::copy(axisStates.begin(), axisStates.end(), axisCache.begin());
}
void PS3::setCache(const std::vector<int> &button, const std::vector<int> &axis) {
  buttonCache.assign(button.begin(), button.end());
  axisCache.assign(axis.begin(), axis.end());
}
void PS3RobotControl::parseArmWorldControlSignal() {
  if (input->getAxis(AXIS_LEFT_X) || input->getAxis(AXIS_LEFT_Y) || input->getKey(BUTTON_UP) ||
      input->getKey(BUTTON_DOWN)) {
    std::array<float, 3> vec = {0, 0, 0};
    vec[1] = -input->getAxisValue(AXIS_LEFT_X);
    vec[0] = -input->getAxisValue(AXIS_LEFT_Y);
    vec[2] = input->getKey(BUTTON_UP) ? 1 : 0;
    vec[2] = input->getKey(BUTTON_DOWN) ? -1 : vec[2];
//    arm_cartesian->moveRelative(vec, WorldTranslate, continuous);
  } else if (input->getAxis(AXIS_RIGHT_X) || input->getAxis(AXIS_RIGHT_Y) ||
             input->getKey(BUTTON_TRIANGLE) || input->getKey(BUTTON_X)) {
    std::array<float, 3> vec = {0, 0, 0};
    vec[0] = input->getAxisValue(AXIS_RIGHT_X);
    vec[1] = input->getAxisValue(AXIS_RIGHT_Y);
    vec[2] = input->getKey(BUTTON_TRIANGLE) ? 1 : 0;
    vec[2] = input->getKey(BUTTON_X) ? -1 : vec[2];
//    arm_cartesian->moveRelative(vec, WorldRotate, continuous);
  } else if (input->getKey(BUTTON_L1)) {
//    gripper->moveJoint(gripperJoints, gripper_velocity);
  } else if (input->getKey(BUTTON_R1)) {
//    gripper->moveJoint(gripperJoints, -gripper_velocity);
    grasped = false;
  } else {
    activated = false;
  }
  if (grasped) {
//    gripper->moveJoint(gripperJoints, gripper_velocity);
  }
}
void PS3RobotControl::parseArmLocalControlSignal() {
  if (input->getAxis(AXIS_LEFT_X) || input->getAxis(AXIS_LEFT_Y) || input->getKey(BUTTON_UP) ||
      input->getKey(BUTTON_DOWN)) {
    std::array<float, 3> vec = {0, 0, 0};
    vec[1] = -input->getAxisValue(AXIS_LEFT_X);
    vec[2] = -input->getAxisValue(AXIS_LEFT_Y);
    vec[0] = input->getKey(BUTTON_UP) ? 1 : 0;
    vec[0] = input->getKey(BUTTON_DOWN) ? -1 : vec[0];
//    arm_cartesian->moveRelative(vec, LocalTranslate, continuous);
  } else if (input->getAxis(AXIS_RIGHT_X) || input->getAxis(AXIS_RIGHT_Y) ||
             input->getKey(BUTTON_TRIANGLE) || input->getKey(BUTTON_X)) {
    std::array<float, 3> vec = {0, 0, 0};
    vec[1] = input->getAxisValue(AXIS_RIGHT_X);
    vec[2] = input->getAxisValue(AXIS_RIGHT_Y);
    vec[0] = input->getKey(BUTTON_TRIANGLE) ? 1 : 0;
    vec[0] = input->getKey(BUTTON_X) ? -1 : vec[0];
//    arm_cartesian->moveRelative(vec, LocalRotate, continuous);
  } else if (input->getKey(BUTTON_L1)) {
//    gripper->moveJoint(gripperJoints, gripper_velocity);
  } else if (input->getKey(BUTTON_R1)) {
    grasped = false;
//    gripper->moveJoint(gripperJoints, -gripper_velocity);
  } else {
    activated = false;
  }
  if (grasped) {
//    gripper->moveJoint(gripperJoints, gripper_velocity);
  }
}
void PS3RobotControl::parseEndingSignal() {
  if (input->getKey(BUTTON_LEFT) && input->getKey(BUTTON_CIRCLE)) {
    throw std::runtime_error("PS3 user interrupt.");
  }
  if (input->getKey(BUTTON_R2) && input->getKey(BUTTON_CIRCLE)) {
    if (!recordLastStep) {
      recordCurrentStep = true;
      std::cout << "Record this step" << std::endl;
    } else {
      recordCurrentStep = false;
    }
    recordLastStep = true;
  } else {
    recordCurrentStep = false;
    recordLastStep = false;
  }
  if (input->getKey(BUTTON_R2) && input->getKey(BUTTON_X)) {
    stopRecord = !stopLastStep;
    stopLastStep = true;
  } else {
    stopLastStep = false;
    stopRecord = false;
  }
  if (input->getKey(BUTTON_L2)) {
    grasped = true;
  }
  continuous = activated;
}
//PS3RobotControl::PS3RobotControl(ControllerManger *m) {
PS3RobotControl::PS3RobotControl() {
//  manger = m;
//  time_step = manger->timeStep;
//  currentPose = manger->wrapper->articulation->get_links()[0]->getGlobalPose();
  input = std::make_unique<PS3>();
}
void PS3RobotControl::step() {
  switch (input->mode) {
  default:
    input->saveCache();
    break;
  case REPLAY:
    break;
  }
  activated = true;
  if (input->getKey(BUTTON_SELECT)) {
    if (mode != ARM_WORLD) {
      std::cout << "Using mode ARM WORLD" << std::endl;
    }
    mode = ControlMode::ARM_WORLD;
  } else if (input->getKey(BUTTON_START)) {
    if (mode != ARM_LOCAL) {
      std::cout << "Using mode ARM LOCAL" << std::endl;
    }
    mode = ControlMode::ARM_LOCAL;
  } else if (input->getKey(BUTTON_PS3)) {
    if (mode != BODY) {
      std::cout << "Using mode BODY" << std::endl;
    }
    mode = ControlMode::BODY;
  }
}
std::vector<int> PS3RobotControl::get_cache() {
  auto dest = input->exportButtonStates();
  auto src = input->exportAxisStates();
  dest.insert(dest.end(), std::make_move_iterator(src.begin()),
              std::make_move_iterator(src.end()));
  return dest;
}
void PS3RobotControl::set_cache(const std::vector<int> &cache) {
  input->setCache(std::vector<int>(cache.begin(), cache.begin() + PS3_BUTTON_COUNT),
                  std::vector<int>(cache.begin() + PS3_BUTTON_COUNT, cache.end()));
}
void PS3RobotControl::parseGripperControlSignal() {
  if (input->getKey(BUTTON_L1)) {
//    gripper->moveJoint(gripperJoints, gripper_velocity);
  } else if (input->getKey(BUTTON_R1)) {
    grasped = false;
//    gripper->moveJoint(gripperJoints, -gripper_velocity);
  }
  if (grasped) {
//    gripper->moveJoint(gripperJoints, gripper_velocity);
  }
}
void PS3RobotControl::parseRootJointControlSignal() {
  if (input->getKey(BUTTON_UP)) {
//    translation->moveJoint({"z_axis_joint"}, translation_velocity);
  }
  if (input->getKey(BUTTON_DOWN)) {
//    translation->moveJoint({"z_axis_joint"}, -translation_velocity);
  }
  if (input->getAxis(AXIS_LEFT_X)) {
    float dir_x = -input->getAxisValue(AXIS_LEFT_X);
//    translation->moveJoint({"y_axis_joint"}, translation_velocity * dir_x);
  } else if (input->getAxis(AXIS_LEFT_Y)) {
    float dir_y = -input->getAxisValue(AXIS_LEFT_Y);
//    translation->moveJoint({"x_axis_joint"}, translation_velocity * dir_y);
  }
  if (input->getKey(BUTTON_TRIANGLE)) {
//    rotation->moveJoint({"r_rotation_joint"}, rotation_velocity);
  }
  if (input->getKey(BUTTON_X)) {
//    rotation->moveJoint({"r_rotation_joint"}, -rotation_velocity);
  }
  if (input->getAxis(AXIS_RIGHT_Y)) {
    float dir_y = input->getAxisValue(AXIS_RIGHT_Y);
//    rotation->moveJoint({"y_rotation_joint"}, rotation_velocity * dir_y);
  }
  if (input->getAxis(AXIS_RIGHT_X)) {
    float dir_x = input->getAxisValue(AXIS_RIGHT_X);
//    rotation->moveJoint({"p_rotation_joint"}, rotation_velocity * dir_x);
  }
}
} // namespace sapien::robot
