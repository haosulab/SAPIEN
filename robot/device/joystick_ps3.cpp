#include "joystick_ps3.h"
#include <cassert>

namespace sapien::robot {

PS3::PS3() : InputDevice("SHANWAN PS3 GamePad") {

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
  }
}
void PS3::runThread() {
  js_event_t event{};
  while (!shouldExit) {
    device.read(reinterpret_cast<char *>(&event), sizeof(js_event_t));
    if (event.type == 1) {
      assert(event.id >= 0 && event.id < PS3_BUTTON_COUNT);
      buttonStates[event.id] = event.action;
    } else if (event.type == 2) {
      assert(event.id >= 0 && event.id < PS3_AXIS_COUNT);
      axisStates[event.id] = event.action;
    }
  }
}
void PS3::shutdown() {
  shouldExit = true;
  worker.join();
}
bool PS3::getKey(ButtonId id) { return buttonStates[id]; }
bool PS3::getAxis(AxisId id) { return axisStates[id]; }
float PS3::getAxisValue(AxisId id) { return static_cast<float>(axisStates[id]) / AXIS_CONST; }
} // namespace sapien::robot
