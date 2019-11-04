//
// Created by sim on 10/12/19.
//

#pragma once

#include "device/joystick_ps3.h"
#include "xarm6.h"
namespace sapien::robot {

class XArm6PS3 : public XArm6 {

public:
  std::unique_ptr<PS3> input;
  explicit XArm6PS3(ControllerManger *manger);
  ~XArm6PS3() override;
  void step();
  inline void set_mode(PS3Mode option) { input->setMode(option); };
  void set_cache(const std::vector<int> &cache);
  std::vector<int> get_cache();
};
} // namespace sapien::robot
