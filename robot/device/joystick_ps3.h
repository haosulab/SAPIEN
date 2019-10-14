//
// Created by sim on 10/11/19.
//
#pragma once

#include "InputDevice.h"
#include <atomic>
#include <cstring>
#include <fcntl.h>
#include <iostream>
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
  AXiS_LEFT_Y,
  AXIS_LEFT_L2,
  AXIS_RIGHT_X,
  AXiS_RIGHT_Y,
  AXIS_RIGHT_L2,
};
#define PS3_AXIS_COUNT 6

struct js_event_t {
  uint32_t time;
  int16_t action;
  uint8_t type;
  uint8_t id;
};
class PS3 : public InputDevice {
protected:
  std::thread worker;
  std::array<std::atomic<int>, PS3_BUTTON_COUNT> buttonStates;
  std::array<std::atomic<int16_t>, PS3_AXIS_COUNT> axisStates;
  void runThread();
  const float AXIS_CONST = 32768;

public:
  PS3();
  void shutdown();
  bool getKey(ButtonId id);
  bool getAxis(AxisId id);
  float getAxisValue(AxisId id);
};
} // namespace sapien::robot
