#pragma once
#include <atomic>
#include <cassert>
#include <fstream>
#include <iostream>
#include <iterator>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace PS3 {

struct js_event_t {
  uint32_t time;
  int16_t action;
  uint8_t type;
  uint8_t id;
};

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

const char *ButtonName[] = {"X",  "circle", "triangle", "square", "l1",   "r1",
                            "l2", "r2",     "select",   "start",  "ps3",  "lj",
                            "rj", "up",     "down",     "left",   "right"};

const char *ActionName[] = {"release", "press"};

struct InputDevice {
  std::string name;
  std::vector<std::string> handlers;
};

bool readDeviceProperty(std::string const &line, InputDevice &device) {
  if (line.empty()) {
    return false;
  }
  switch (line[0]) {
  case 'N':
    device.name = line.substr(9, line.length() - 10);
    break;
  case 'H': {
    std::stringstream ss(line.substr(12));
    std::string handler = "";
    while (!ss.eof()) {
      ss >> handler;
      if (!handler.empty()) {
        device.handlers.push_back(handler);
      }
    }
  } break;
  default:
    break;
  }
  return true;
}

std::vector<InputDevice> getDevices() {
  std::vector<InputDevice> devices;
  std::ifstream s("/proc/bus/input/devices");
  std::string line;

  while (std::getline(s, line)) {
    devices.push_back({});
    while (std::getline(s, line)) {
      if (!readDeviceProperty(line, devices.back())) {
        break;
      }
    }
  }
  return devices;
}

std::string findPS3Controller() {
  for (auto d : getDevices()) {
    if (d.name == "SHANWAN PS3 GamePad") {
      for (auto &handler : d.handlers) {
        if (handler.substr(0, 2) == "js") {
          return handler;
        }
      }
    }
  }
  return "";
}

class PS3Input {
  std::ifstream device;
  std::mutex readMutex;
  std::thread worker;
  std::atomic<bool> shouldExit;
  std::array<std::atomic<int>, PS3_BUTTON_COUNT> buttonStates;

public:
  PS3Input() {
    std::string deviceName = findPS3Controller();
    if (deviceName.empty()) {
      throw std::runtime_error("No PS3 Controller found!");
    }
    device = std::ifstream("/dev/input/" + deviceName, std::ios::in | std::ios::binary);
    for (auto &state : buttonStates) {
      state = 0;
    }
    worker = std::thread(&PS3Input::runThread, this);
  }

  void runThread() {
    js_event_t event;
    while (!shouldExit) {
      device.read(reinterpret_cast<char *>(&event), sizeof(js_event_t));
      if (event.type == 1) {
        assert(event.id >= 0 && event.id < PS3_BUTTON_COUNT);
        buttonStates[event.id] = event.action;
      }
    }
  }

  bool getKey(ButtonId id) {
    assert(id >= 0 && id < PS3_BUTTON_COUNT);
    return buttonStates[id];
  }

  void shutdown() {
    shouldExit = true;
    worker.join();
  }
};

} // namespace PS3
