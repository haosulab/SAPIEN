//
// Created by sim on 10/20/19.
//

#pragma once
#include "simulation.h"

namespace sapien::robot{

class DemonstrationSimulation: public Simulation{
  void setRenderer(class DemonstrationGUI *renderer);
};

}
