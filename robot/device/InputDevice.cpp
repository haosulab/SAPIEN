//
// Created by sim on 10/11/19.
//

#include "InputDevice.h"
#include <sstream>

std::vector<Device> getDevices() {
  std::vector<Device> devices;
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
bool readDeviceProperty(std::string const &line, Device &device) {
  if (line.empty()) {
    return false;
  }
  switch (line[0]) {
  case 'N':
    device.name = line.substr(9, line.length() - 10);
    break;
  case 'H': {
    std::stringstream ss(line.substr(12));
    while (!ss.eof()) {
      std::string handler;
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
InputDevice::InputDevice(const std::string &deviceName) {
  std::string filename = findDevice(deviceName);
  if (filename.empty()) {
    std::cerr << "No " << deviceName << " found!" << std::endl;
  }
  device = std::ifstream("/dev/input/" + filename, std::ios::in | std::ios::binary);
  shouldStart = true;
}
std::string InputDevice::findDevice(const std::string &deviceName) {
  for (const auto &d : getDevices()) {
    if (d.name == deviceName) {
      for (auto &handler : d.handlers) {
        if (handler.substr(0, 2) == "js") {
          return handler;
        }
      }
    }
  }
  return "";
}
