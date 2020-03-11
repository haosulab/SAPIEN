#include "renderer/optifuser_renderer.h"
#include "sapien_scene.h"
#include "simulation.h"
#include "common.h"

#include "catch.hpp"

using namespace sapien;

TEST_CASE("Creation and Shutdown", "[simulation]") {
  Simulation sim;
  auto scene = sim.createScene();
  scene->setTimestep(1 / 20.f);

  REQUIRE_NO_ERROR(sim);
}

TEST_CASE("Multiple scenes", "[simulation]") {
  Simulation sim;

  auto s0 = sim.createScene();
  auto s1 = sim.createScene({0, 0, 1}, PxSolverType::eTGS);
  auto s2 = sim.createScene({0, 0, 1}, PxSolverType::ePGS, PxSceneFlag::eENABLE_CCD);

  REQUIRE_NO_ERROR(sim);
}

TEST_CASE("Create renderer", "[simulation]") {
  Simulation sim;
  Renderer::OptifuserRenderer renderer;
  sim.setRenderer(&renderer);

  auto s0 = sim.createScene();

  REQUIRE_NO_ERROR(sim);
}

TEST_CASE("Create material", "[simulation]") {
  Simulation sim;
  auto s0 = sim.createPhysicalMaterial(0.2, 0.1, 0.5);

  REQUIRE(s0->getStaticFriction() - 0.2 < 1e-8);
  REQUIRE(s0->getDynamicFriction() - 0.1 < 1e-8);
  REQUIRE(s0->getRestitution() - 0.5 < 1e-8);

  REQUIRE_NO_ERROR(sim);
}
