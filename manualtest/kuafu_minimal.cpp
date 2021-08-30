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
  auto renderer = std::make_shared<Renderer::KuafuRenderer>(true);
  renderer->setAssetsPath("../3rd_party/kuafu/resources");
  sim->setRenderer(renderer);
  renderer->init();

  auto scene = sim->createScene();
  scene->setTimestep(1 / 60.f);
  auto ground_material = renderer->createMaterial();
  ground_material->setBaseColor({1.0, 0.5, 0.5, 1.0});
//  ground_material->setRoughness(1000.0f);
//  std::dynamic_pointer_cast<Renderer::KuafuMaterial>(ground_material)->getKMaterial()->emission = glm::vec3(1.0F);
//  ground_material->setSpecular(0.0f);
  scene->addGround(0, true, nullptr, ground_material);


//  // axes
//  auto axes_builder = scene->createActorBuilder();
//  axes_builder->addBoxVisual(
//      {{1, 0, 0.}, PxIdentity}, {1, 0.1, 0.1}, {1.0, 0.0, 0.0});
//  axes_builder->addBoxVisual(
//      {{0, 1, 0.}, PxIdentity}, {0.1, 1, 0.1}, {0.0, 1.0, 0.0});
//  axes_builder->addBoxVisual(
//      {{0, 0, 1}, PxIdentity}, {0.1, 0.1, 1}, {0.0, 0.0, 1.0});
//  auto axes = axes_builder->build(true);
//  axes->setPose({{0, 0, 0.05}, PxIdentity});


//  auto copper = renderer->createMaterial();
//  copper->setBaseColor({0.2, 0.4, 0.1, 1});
////  copper->setMetallic(1.f);
//  copper->setRoughness(1000.f);
////  copper->setSpecular(0.5f);

  scene->setAmbientLight({0.5, 0.5, 0.5});
//  scene->addDirectionalLight({0, 0, 0}, {100.0, 100.0, 100.0}, false, {0, 0, 2}, 0, 0, 0);
//  scene->addSpotLight({-2, 0, 2}, {0, 0, 0}, 0, 0, {1000., 1000., 1000.}, false, 0, 0);

  auto mount = scene->createActorBuilder()->build(true);
//  mount->setPose(PxTransform({-5, 0.6, 0.5}, {}));
  mount->setPose(PxTransform({0.6, 5, 0.5}, {0, 0, -0.7071068, 0.7071068}));
  auto cam = scene->addMountedCamera(
      "cam", mount, PxTransform({0, 0, 0}, PxIdentity), 800, 600, 0, 1.0);

  auto copper = renderer->createMaterial();
  copper->setBaseColor({0.7, 0.6, 0.2, 1});
//  copper->setMetallic(1.f);
  copper->setRoughness(5.f);
  copper->setMetallic(1.f);

//  auto sphere_builder = scene->createActorBuilder();
//  sphere_builder->addSphereVisualWithMaterial(
//      {{0, 0, 0.}, PxIdentity}, 1.f, copper);
//  sphere_builder->addSphereShape(
//      {{0, 0, 0.}, PxIdentity}, 1.f);
//  auto sphere = sphere_builder->build(false);
//  sphere->setPose({{0, 4, 4.}, PxIdentity});
////  sphere->lockMotion(false, false, false, false, false, false);

  auto obj_builder = scene->createActorBuilder();
  obj_builder->addVisualFromFile(
      "/zdata/ssource/ICCV2021_Diagnosis/ocrtoc_materials/models/camera/visual_mesh.obj");
  obj_builder->addMultipleConvexShapesFromFile(
      "/zdata/ssource/ICCV2021_Diagnosis/ocrtoc_materials/models/camera/collision_mesh.obj");
  auto obj = obj_builder->build(false);
  obj->setPose({{0, 0, 3.}, PxIdentity});

//  auto cap_builder = scene->createActorBuilder();
//  cap_builder->addCapsuleVisual(
//      {{0, 0, 0.}, PxIdentity}, 0.3, 0.3, {1.0, 0.4, 0.2});
//  cap_builder->addCapsuleShape(
//      {{0, 0, 0.}, PxIdentity}, 0.3, 0.3);
//  auto cap = cap_builder->build(false);
//  cap->setPose({{0, 0, 2.}, PxIdentity});

  auto cube_builder = scene->createActorBuilder();
  cube_builder->addBoxVisual(
      {{0, 0, 0.}, PxIdentity}, {0.1, 0.1, 0.1}, {1.0, 1.0, 1.0});
  cube_builder->addBoxShape(
      {{0, 0, 0.}, PxIdentity}, {0.1, 0.1, 0.1});
  auto cube = cube_builder->build(false);
  cube->setPose({{0, 0, 5}, PxIdentity});


  while (true) {
    scene->step();
    scene->updateRender();
    cam->takePicture();
    auto a = cam->getColorRGBA();
    std::cout << a[640*320+320] << std::endl;
  }
}