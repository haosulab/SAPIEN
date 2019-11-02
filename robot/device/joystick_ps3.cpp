#include "joystick_ps3.h"
#include <cassert>

namespace sapien::robot {

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
void PS3::setMode(PS3Mode option) {
  mode = option;
}
void PS3::saveCache() {
  std::lock_guard<std::mutex> guard(lock);
  std::copy(buttonStates.begin(), buttonStates.end(), buttonCache.begin());
  std::copy(axisStates.begin(), axisStates.end(), axisCache.begin());
}
void PS3::setCache(const std::vector<int> &button, const std::vector<int> &axis) {
  buttonCache.assign(button.begin(), button.end());
  axisCache.assign(axis.begin(), axis.end());
}
} // namespace sapien::robot
