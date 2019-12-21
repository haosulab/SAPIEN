#include "simulation.h"
#define CATCH_CONFIG_MAIN
#include "catch.hpp"

using namespace sapien;

TEST_CASE("Creation and Shutdown", "[simulation]") {
  Simulation sim;
  sim.setTimestep(1.f / 60.f);
  sim.addGround(-1);
}
