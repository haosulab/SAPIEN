#include "sapien/component/sapien_renderer/sapien_renderer.h"
#include <svulkan2/scene/loader.h>

namespace sapien {
namespace component {

std::vector<std::shared_ptr<Entity>> LoadScene(std::string const &filename) {
  std::vector<std::shared_ptr<Entity>> entities;

  auto scene = svulkan2::scene::LoadScene(filename);

  for (auto obj : scene->getObjects()) {
    for (auto shape : obj->getModel()->getShapes()) {
      auto entity = std::make_shared<Entity>();
      auto body = std::make_shared<SapienRenderBodyComponent>();

      std::vector<std::shared_ptr<RenderShapeTriangleMeshPart>> parts = {
          std::make_shared<RenderShapeTriangleMeshPart>(shape)};

      body->setName(shape->name);

      body->attachRenderShape(std::make_shared<RenderShapeTriangleMesh>(parts));
      entity->addComponent(body);

      auto &T = obj->getTransform();
      entity->setPose({{T.position.x, T.position.y, T.position.z},
                       {T.rotation.w, T.rotation.x, T.rotation.y, T.rotation.z}});
      entity->setName("Visual Mesh: " + shape->name);
      entities.push_back(entity);
    }
  }

  // for (auto obj : scene->getObjects()) {
  //   auto entity = std::make_shared<Entity>();
  //   auto body = std::make_shared<SapienRenderBodyComponent>();
  //   std::vector<std::shared_ptr<RenderShapeTriangleMeshPart>> parts;
  //   for (auto shape : obj->getModel()->getShapes()) {
  //     parts.push_back(std::make_shared<RenderShapeTriangleMeshPart>(shape));

  //   }
  //   body->attachRenderShape(std::make_shared<RenderShapeTriangleMesh>(parts));
  //   entity->addComponent(body);

  //   auto &T = obj->getTransform();
  //   entity->setPose({{T.position.x, T.position.y, T.position.z},
  //                    {T.rotation.w, T.rotation.x, T.rotation.y, T.rotation.z}});
  //   entity->setName("Visual Mesh");
  //   entities.push_back(entity);
  // }

  for (auto l : scene->getPointLights()) {
    auto entity = std::make_shared<Entity>();

    auto color = l->getColor();
    auto &T = l->getTransform();
    Pose glPose = {{T.position.x, T.position.y, T.position.z},
                   {T.rotation.w, T.rotation.x, T.rotation.y, T.rotation.z}};
    Pose pose = glPose * POSE_ROS_TO_GL;

    auto c = std::make_shared<SapienRenderPointLightComponent>();
    c->setColor({color.r, color.g, color.b});
    c->setShadowEnabled(true);
    c->setLocalPose({});

    entity->addComponent(c);
    entity->setName("Point Light");
    entity->setPose(pose);
    entities.push_back(entity);
  }

  for (auto l : scene->getDirectionalLights()) {
    auto entity = std::make_shared<Entity>();

    auto color = l->getColor();
    auto &T = l->getTransform();
    Pose glPose = {{T.position.x, T.position.y, T.position.z},
                   {T.rotation.w, T.rotation.x, T.rotation.y, T.rotation.z}};
    Pose pose = glPose * POSE_ROS_TO_GL;

    auto c = std::make_shared<SapienRenderDirectionalLightComponent>();
    c->setColor({color.r, color.g, color.b});
    c->setShadowEnabled(true);
    c->setLocalPose({});

    entity->addComponent(c);
    entity->setName("Directional Light");
    entity->setPose(pose);
    entities.push_back(entity);
  }

  for (auto l : scene->getSpotLights()) {
    auto entity = std::make_shared<Entity>();

    auto color = l->getColor();
    auto &T = l->getTransform();
    Pose glPose = {{T.position.x, T.position.y, T.position.z},
                   {T.rotation.w, T.rotation.x, T.rotation.y, T.rotation.z}};
    Pose pose = glPose * POSE_ROS_TO_GL;

    auto c = std::make_shared<SapienRenderSpotLightComponent>();
    c->setColor({color.r, color.g, color.b});
    c->setShadowEnabled(true);
    c->setFovInner(l->getFovSmall());
    c->setFovOuter(l->getFov());
    c->setLocalPose({});

    entity->addComponent(c);
    entity->setName("Spot Light");
    entity->setPose(pose);
    entities.push_back(entity);
  }

  for (auto l : scene->getParallelogramLights()) {
    auto entity = std::make_shared<Entity>();

    auto color = l->getColor();
    auto &T = l->getTransform();
    Pose glPose = {{T.position.x, T.position.y, T.position.z},
                   {T.rotation.w, T.rotation.x, T.rotation.y, T.rotation.z}};
    Pose pose = glPose * POSE_ROS_TO_GL;

    auto c = std::make_shared<SapienRenderParallelogramLightComponent>();
    c->setColor({color.r, color.g, color.b});
    c->setShadowEnabled(true);
    c->setShape(l->getHalfSize().x, l->getHalfSize().y, l->getAngle());
    c->setLocalPose({});

    entity->addComponent(c);
    entity->setName("Area Light");
    entity->setPose(pose);
    entities.push_back(entity);
  }

  return entities;
}

} // namespace component
} // namespace sapien
