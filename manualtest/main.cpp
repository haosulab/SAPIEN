#include "sapien/component/physx/collision_shape.h"
#include "sapien/component/physx/physical_material.h"
#include "sapien/component/physx/physx_system.h"
#include "sapien/component/physx/rigid_component.h"
#include "sapien/component/sapien_renderer/material.h"
#include "sapien/component/sapien_renderer/render_body_component.h"
#include "sapien/component/sapien_renderer/sapien_renderer_system.h"
#include "sapien/component/sapien_renderer/render_shape.h"
#include "sapien/component/sapien_renderer/window.h"
#include "sapien/entity.h"
#include "sapien/scene.h"
using namespace sapien;

int main(int argc, char *argv[]) {
  auto scene = std::make_shared<Scene>();
  scene->addSystem(std::make_shared<component::PhysxSystem>());
  scene->addSystem(std::make_shared<component::SapienRendererSystem>());

  // entity
  auto entity = std::make_shared<Entity>();
  entity->setName("ground");

  // rigid body component
  auto staticComponent = std::make_shared<component::PhysxRigidStaticComponent>();

  // collision
  auto mat = std::make_shared<component::PhysxMaterial>(0.3, 0.3, 0.1);
  auto shape = std::make_shared<component::PhysxCollisionShapePlane>(mat);
  shape->setLocalPose({{0.f, 0.f, 0.f}, {0.7071068, 0, -0.7071068, 0}});

  // add collision to component
  staticComponent->attachCollision(std::move(shape));

  // add rigid body component to entity
  entity->addComponent(staticComponent);

  auto groundVisual = std::make_shared<component::SapienRenderBodyComponent>();
  auto render_mat = std::make_shared<component::SapienRenderMaterial>(
      std::array<float, 4>{0.f, 0.f, 0.f, 1.f}, std::array<float, 4>{0.8f, 0.8f, 0.8f, 1.f}, 0.1f,
      0.7f, 0.f, 0.f, 1.1f, 1.f);
  auto plane = std::make_shared<component::RenderShapePlane>(Vec3{1.f, 10.f, 10.f}, render_mat);
  groundVisual->attachVisualShape(std::move(plane));

  entity->addComponent(groundVisual);

  // spawn
  scene->addEntity(entity);

  auto renderSystem = scene->getSystem<component::SapienRendererSystem>();

  component::SapienRendererWindow window(1920, 1080, "../vulkan_shader/trivial");
  window.setScene(scene);

  while (1) {
    renderSystem->step();
    window.render("Color");
  }
}
