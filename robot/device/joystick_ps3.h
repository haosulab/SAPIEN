//
// Created by sim on 10/11/19.
//
#pragma once

#include "input_device.h"
#include <atomic>
#include <controller/controller_manger.h>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <unistd.h>
#include <vector>

namespace sapien::robot {

enum ButtonId {
  BUTTON_X,
  BUTTON_CIRCLE,
  BUTTON_TRIANGLE,
  BUTTON_SQUARE,
  BUTTON_L1,
  BUTTON_R1,
  BUTTON_L2,
  BUTTON_R2,
  BUTTON_SELECT,
  BUTTON_START,
  BUTTON_PS3,
  BUTTON_LEFT_JOYSTICK,
  BUTTON_RIGHT_JOYSTICK,
  BUTTON_UP,
  BUTTON_DOWN,
  BUTTON_LEFT,
  BUTTON_RIGHT
};
#define PS3_BUTTON_COUNT 17

enum AxisId {
  AXIS_LEFT_X,
  AXIS_LEFT_Y,
  AXIS_LEFT_L2,
  AXIS_RIGHT_X,
  AXIS_RIGHT_Y,
  AXIS_RIGHT_L2,
};
#define PS3_AXIS_COUNT 6

enum PS3Mode { NORMAL, DEMONSTRATION, REPLAY };

struct js_event_t {
  uint32_t time;
  int16_t action;
  uint8_t type;
  uint8_t id;
};
class PS3 : public InputDevice {
protected:
  std::thread worker;
  std::array<int, PS3_BUTTON_COUNT> buttonStates{};
  std::array<int, PS3_AXIS_COUNT> axisStates{};
  std::mutex lock;
  void runThread();
  const float AXIS_CONST = 32768;

  // This cache is only used in demonstration mode to allow exact reproducible result
  std::vector<int> buttonCache;
  std::vector<int> axisCache;

public:
  PS3Mode mode = PS3Mode::NORMAL;
  PS3();
  void setMode(PS3Mode);
  void shutdown();
  bool getKey(ButtonId id);
  bool getAxis(AxisId id);
  float getAxisValue(AxisId id);
  inline std::vector<int> exportButtonStates() { return buttonCache; };
  inline std::vector<int> exportAxisStates() { return axisCache; };
  void saveCache();
  void setCache(const std::vector<int> &button, const std::vector<int> &axis);
};

class PS3RobotControl {
protected:
  ControllerManger *manger;
  std::unique_ptr<PS3> input;

  bool reset = true;
  bool continuous = true;
  bool activated = false;
  bool startRecord = false;
  ControlMode mode = ControlMode::BODY;

  std::vector<std::string> gripperJoints = {};
  JointVelocityController *gripper = nullptr;
  CartesianVelocityController *arm_cartesian = nullptr;
  JointVelocityController *translation = nullptr;
  JointVelocityController *rotation = nullptr;

  float gripper_velocity = 0;
  bool grasped = false;
  float arm_velocity = 0;
  float arm_angular_velocity = 0;

  PxVec3 pos = {0, 0, 0};
  PxReal angle = 0;
  float translation_velocity = 0;
  float rotation_velocity = 0;
  float time_step;

protected:
  void parseRootJointControlSignal();
  void parseArmWorldControlSignal();
  void parseArmLocalControlSignal();
  void parseGripperControlSignal();
  void parseEndingSignal();

public:
  explicit PS3RobotControl(ControllerManger *m);
  ~PS3RobotControl() { input->shutdown(); }
  virtual void step();
  std::vector<int> get_cache();
  void set_cache(const std::vector<int> &cache);

  // Inline functions
  inline bool start_record() { return startRecord; };
  inline void set_gripper_velocity(float v) { gripper_velocity = v; }
  inline void set_arm_velocity(float v) {
    arm_cartesian->setVelocity(v);
    arm_velocity = v;
  }
  inline void set_arm_angular_velocity(float v) {
    arm_cartesian->setAngularVelocity(v);
    arm_angular_velocity = v;
  }
  inline float get_gripper_velocity() { return gripper_velocity; }
  inline float get_arm_velocity() { return arm_velocity; }
  inline float get_arm_angular_velocity() { return arm_angular_velocity; }
  inline void set_mode(PS3Mode option) { input->setMode(option); };
  inline void set_translation_velocity(float v) { translation_velocity = v; };
  inline void set_rotation_velocity(float v) { rotation_velocity = v; };
};

} // namespace sapien::robot
