#include "sapien/articulation/articulation_builder.h"
#include "sapien/articulation/sapien_articulation.h"
#include "sapien/articulation/sapien_link.h"
#include "sapien/renderer/server/client.h"
#include "sapien/sapien_actor.h"
#include "sapien/sapien_drive.h"
#include "sapien/sapien_scene.h"
#include "sapien/simulation.h"

int main() {
  auto sim = sapien::Simulation::getInstance();
  std::string address = "localhost:5003";
  auto renderer = std::make_shared<sapien::Renderer::server::ClientRenderer>(address);
  sim->setRenderer(renderer);
  auto scene = sim->createScene();
}
