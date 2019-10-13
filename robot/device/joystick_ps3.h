//
// Created by sim on 10/11/19.
//
#pragma once

#include "InputDevice.h"
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>
#include <vector>

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
  AXIS_LEFT_STICK_HORIZONTAL,  // 0
  AXIS_LEFT_STICK_VERTICAL,    // 1
  AXIS_RIGHT_STICK_HORIZONTAL, // 2
  AXIS_RIGHT_STICK_VERTICAL,   // 3
  NA4,
  NA5,
  NA6,
  NA7,
  AXIS_DPAD_UP,       // 8
  AXIS_DPAD_RIGHT,    // 9
  AXIS_DPAD_DOWN,     // 10
                      // who knows what the left value should be...
  AXIS_DPAD_LEFT,     // 11
  AXIS_LEFT_TRIGGER,  // 12
  AXIS_RIGHT_TRIGGER, // 13
  AXIS_LEFT_BUMPER,   // 14
  AXIS_RIGHT_BUMPER,  // 15
  AXIS_TRIANGLE,      // 16
  AXIS_CIRCLE,        // 17
  AXIS_X,             // 18
  AXIS_SQUARE,        // 19
  NA20,
  NA21,
  NA22,
  // X is left/right
  AXIS_ACCEL_X, // 23 note: left is positive, right is negative
                // Y is front/back
  AXIS_ACCEL_Y, // 24 note: back is positive, forward is negative
                // Z is up/down
  AXIS_ACCEL_Z, // 25 note: can't tell what sign is what
};
#define AXIS_COUNT 26

#define EVENT_BUTTON 0x01 // Button state update
#define EVENT_AXIS 0x02   // Axis state update
#define EVENT_INIT 0x80   // Non-input event, to be ignored
#define JS_EVENT_SIZE 8

struct js_event_t {
  uint32_t time;
  int16_t action;
  uint8_t type;
  uint8_t id;
};
class PS3Input : public InputDevice {
public:
  PS3Input();
};
