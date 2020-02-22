//
// Created by zack on 2/21/20.
//
#include "urdf/ros_urdf_loader.h"
#include "articulation/articulation_builder.h"
#include "articulation/sapien_kinematic_articulation.h"
#include "articulation/sapien_kinematic_joint.h"
#include "articulation/urdf_loader.h"
#include "renderer/optifuser_controller.h"
#include "renderer/optifuser_renderer.h"
#include "sapien_scene.h"
#include "simulation.h"
#include <thread>

using namespace sapien::ros2;
using namespace sapien;

int main(int argc, char *argv[]) {
  Simulation sim;
  Renderer::OptifuserRenderer renderer;
  sim.setRenderer(&renderer);
  Renderer::OptifuserController controller(&renderer);

  controller.showWindow();

  auto s0 = sim.createScene();
  s0->setTimestep(1 / 480.f);
  s0->addGround(-1);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("ros_test");

  auto loader = ROS_urdf_Loader(node, s0.get(), "sapien_resources", "panda_urdf_description/urdf/panda.urdf");

  auto a = loader.loadArticulation(nullptr);

  a->setRootPose({{0, 0, -1}, PxIdentity});

  s0->setAmbientLight({0.3, 0.3, 0.3});
  s0->setShadowLight({0, -1, -1}, {.5, .5, 0.4});

  for (auto j : a->getBaseJoints()) {
    static_cast<SKJoint *>(j)->setDriveProperties(1, 1, 1);
  }
//  a->setDriveTarget({0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1});

  controller.setCameraPosition(-5, 0, 0);
  controller.setCurrentScene(s0.get());


  while (!controller.shouldQuit()) {
    for (int i = 0; i < 8; ++i) {
      s0->step();
    }
    s0->updateRender();
    controller.render();
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}