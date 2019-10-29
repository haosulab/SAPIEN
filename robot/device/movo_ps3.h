//
// Created by sim on 10/12/19.
//

#pragma once

#include "device/joystick_ps3.h"
#include "device/movo.h"
namespace sapien::robot {

class MOVOPS3 : public MOVO {

public:
  std::unique_ptr<PS3> input;
  explicit MOVOPS3(ControllerManger *manger);
  ~MOVOPS3() override;
  void step();
  inline void set_mode(PS3Mode option) { input->setMode(option); };
  void set_cache(const std::vector<int> &cache);
  std::vector<int> get_cache();
};
} // namespace sapien::robot
