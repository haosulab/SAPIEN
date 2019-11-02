//
// Created by sim on 10/31/19.
//

#pragma once

#include "device/joystick_ps3.h"
#include "device/single_gripper.h"
namespace sapien::robot {

class KinovaGripperPS3 : public SingleKinovaGripper {

public:
  std::unique_ptr<PS3> input;
  explicit KinovaGripperPS3(ControllerManger *manger);
  ~KinovaGripperPS3() override;
  void step();
  inline void set_mode(PS3Mode option) { input->setMode(option); };
  void set_cache(const std::vector<int> &cache);
  std::vector<int> get_cache();
  inline bool start_record() { return startRecord; };
};
} // namespace sapien::robot
