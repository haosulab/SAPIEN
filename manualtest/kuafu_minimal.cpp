#include "articulation/articulation_builder.h"
#include "articulation/sapien_articulation.h"
#include "articulation/sapien_link.h"
#include "renderer/kuafu_renderer.hpp"
#include "sapien_actor.h"
#include "sapien_drive.h"
#include "sapien_scene.h"
#include "simulation.h"

using namespace sapien;

int main() {
  auto sim = std::make_shared<Simulation>();
  auto renderer = std::make_shared<Renderer::KuafuRenderer>(false);
  renderer->setAssetsPath("../3rd_party/kuafu/resources");
  sim->setRenderer(renderer);
  renderer->init();

  auto scene = sim->createScene();
  scene->setTimestep(1 / 60.f);
  auto ground_material = renderer->createMaterial();
  ground_material->setBaseColor({1.0, 1.0, 1.0, 1.0});
//  ground_material->setRoughness(1000.0f);
  std::dynamic_pointer_cast<Renderer::KuafuMaterial>(ground_material)->getKMaterial()->emission = glm::vec3(1.0F);
//  ground_material->setSpecular(0.0f);
  scene->addGround(0, true, nullptr, ground_material);

//  auto copper = renderer->createMaterial();
//  copper->setBaseColor({0.2, 0.4, 0.1, 1});
////  copper->setMetallic(1.f);
//  copper->setRoughness(1000.f);
////  copper->setSpecular(0.5f);

  scene->setAmbientLight({0.1, 0.1, 0.1});
//  scene->addDirectionalLight({0, 0, 0}, {100.0, 100.0, 100.0}, false, {0, 0, 2}, 0, 0, 0);
//  scene->addSpotLight({-2, 0, 2}, {0, 0, 0}, 0, 0, {1000., 1000., 1000.}, false, 0, 0);

  auto mount = scene->createActorBuilder()->build(true);
  mount->setPose(PxTransform({-8, 0, 2}, PxIdentity));
  auto cam = scene->addMountedCamera(
      "cam", mount, PxTransform({0, 0, 0}, PxIdentity), 800, 600, 0, 1.0);

  auto copper = renderer->createMaterial();
  copper->setBaseColor({0.7, 0.6, 0.2, 1});
//  copper->setMetallic(1.f);
  copper->setRoughness(5.f);
  copper->setMetallic(1.f);

  auto sphere_builder = scene->createActorBuilder();
  sphere_builder->addSphereVisualWithMaterial(
      {{0, 0, 0.}, PxIdentity}, 1.f, copper);
  sphere_builder->addSphereShape(
      {{0, 0, 0.}, PxIdentity}, 1.f);
  auto sphere = sphere_builder->build(false);
  sphere->setPose({{0, 0, 4.}, PxIdentity});
//  sphere->lockMotion(false, false, false, false, false, false);

//  auto cube_builder = scene->createActorBuilder();
//  cube_builder->addBoxVisual(
//      {{0, 0, 0.}, PxIdentity}, {0.1, 0.1, 0.1}, {1.0, 0.4, 0.2});
//  cube_builder->addBoxShape(
//      {{0, 0, 0.}, PxIdentity}, {0.1, 0.1, 0.1});
//  auto cube = cube_builder->build(true);
//  cube->setPose({{0, 1, 3.}, PxIdentity});


//  auto& _K = renderer->_getK();
//  auto& _KScene = _K.getScene();
//
//  _KScene.setCamera(std::make_shared<Renderer::KCamera>("cam", 800, 600));
//  auto _KCamera = _KScene.getCamera();
//  _KCamera->setPosition(glm::vec3(-5.0F, 0.5F, 10.0F));
//  _KCamera->setFront(glm::vec3(0.67F, 0.0F, -0.8F));


//  // add light
//  auto lightPlane = kuafu::loadObj("../3rd_party/kuafu/resources/models/plane.obj");
//  kuafu::Material lightMaterial;
//  lightMaterial.emission = glm::vec3(10.0F);
//  lightPlane->setMaterial(lightMaterial);
//  auto transform = glm::translate(glm::mat4(1.0F), glm::vec3(0.0F, 0.0F, 80.0F));
//  transform = glm::scale(transform, {10, 10, 10});
//  auto lightPlaneInstance = kuafu::instance(lightPlane, transform);
//  _KScene.submitGeometry(lightPlane);
//  _KScene.submitGeometryInstance(lightPlaneInstance);

//  // add ground
//  auto ground = kuafu::loadObj("../3rd_party/kuafu/resources/models/plane.obj");
//  kuafu::Material groundMaterial;
//  groundMaterial.kd = glm::vec3(1.0F);
//  ground->setMaterial(groundMaterial);
//  transform = glm::scale(glm::mat4(1.0F), {10, 10, 10});
////  transform = glm::mat4(1.0F);
//  auto groundInstance = kuafu::instance(ground, transform);
//  _KScene.submitGeometry(ground);
//  _KScene.submitGeometryInstance(groundInstance);

//  // add sphere
//  auto sphere = kuafu::loadObj("../3rd_party/kuafu/resources/models/sphere.obj");
//  kuafu::Material sphereMaterial;
//  sphereMaterial.kd = glm::vec3(glm::vec3(0.2F, 0.4F, 1.0F));
//  sphere->setMaterial(sphereMaterial);
//  auto transform = glm::mat4(1.0F);
//  transform = glm::translate(transform, {0, 0.0, 0.0});
//  auto sphereInstance = kuafu::instance(sphere, transform);
//  _KScene.submitGeometry(sphere);
//  _KScene.submitGeometryInstance(sphereInstance);

  while (true) {
    scene->step();
    scene->updateRender();
    cam->takePicture();
//    _K.run();
  }
}