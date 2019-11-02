//
// Created by sim on 10/11/19.
//
#pragma once

#include "InputDevice.h"
#include <atomic>
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
} // namespace sapien::robot
