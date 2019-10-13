//
// Created by sim on 10/11/19.
//

#pragma once
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

struct Device {
  std::string name;
  std::vector<std::string> handlers;
};
bool readDeviceProperty(std::string const &line, Device &device);
std::vector<Device> getDevices();
class InputDevice {
  std::ifstream device;
  bool shouldExit = false;

public:
  void init(const std::string &deviceName);
  static std::string findDevice(const std::string &deviceName);
};
