//
// Created by sim on 10/12/19.
//

#pragma once

#include "device/joystick_ps3.h"
#include "device/movo_device_control.h"
namespace sapien::robot {

class MOVOPS3 : public MOVO {

public:
  std::unique_ptr<PS3> input;
  explicit MOVOPS3(ControllerManger *manger);
  ~MOVOPS3() override;
  void step() override;
};
} // namespace sapien::robot
