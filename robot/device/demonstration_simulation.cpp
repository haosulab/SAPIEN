//
// Created by sim on 10/20/19.
//

#include "demonstration_simulation.h"
#include "demonstration_gui.h"

void sapien::robot::DemonstrationSimulation::setRenderer(
    sapien::robot::DemonstrationGUI *renderer) {
  sapien::Simulation::setRenderer(renderer);
  renderer->setSimulation(this);
}
