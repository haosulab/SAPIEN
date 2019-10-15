//
// Created by sim on 10/12/19.
//

#pragma once

#include "device/movo_device_control.h"
#include "optifuser.h"
namespace sapien::robot {

class MOVOKeyboard : public MOVO {
public:
  explicit MOVOKeyboard(ControllerManger *manger);
  void step() override;
};
} // namespace sapien::robot
