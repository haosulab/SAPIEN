#include "sapien/sapien_renderer/sapien_renderer.h"
#include <queue>
#include <svulkan2/scene/loader.h>

namespace sapien {
namespace sapien_renderer {

std::unique_ptr<RenderSceneNode> LoadScene(std::string const &filename, bool applyScale) {
  auto scene = svulkan2::scene::LoadScene(filename);

  std::unique_ptr<RenderSceneNode> rootNode;

  std::queue<svulkan2::scene::Node *> q;
  q.push(&scene->getRootNode());

  std::queue<RenderSceneNode *> qs;
  qs.push(nullptr);

  std::queue<glm::mat4> qt;
  qt.push(glm::mat4(1.f));

  while (!q.empty()) {
    svulkan2::scene::Node *node = q.front();
    q.pop();

    glm::mat4 parentTransform = qt.front();
    qt.pop();

    RenderSceneNode *parent = qs.front();
    qs.pop();

    auto mat = parentTransform * node->getTransform().matrix();

    auto newSnode = std::make_unique<RenderSceneNode>();
    newSnode->name = node->getName();

    auto transform = svulkan2::scene::Transform::FromMatrix(mat);

    newSnode->pose = Pose(
        {transform.position.x, transform.position.y, transform.position.z},
        {transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z});

    if (!applyScale) {
      // stop here
      newSnode->scale = {transform.scale.x, transform.scale.y, transform.scale.z};

      mat = glm::mat4(1.f);
      transform = svulkan2::scene::Transform{};
    } else {
      // apply scale down the tree
      newSnode->scale = {1.f, 1.f, 1.f};

      mat = glm::scale(glm::mat4(1.f), transform.scale);
      transform = svulkan2::scene::Transform{.scale = transform.scale};
    }

    if (auto obj = dynamic_cast<svulkan2::scene::Object *>(node)) {
      for (auto shape : obj->getModel()->getShapes()) {
        std::vector<std::shared_ptr<RenderShapeTriangleMeshPart>> parts = {
            std::make_shared<RenderShapeTriangleMeshPart>(shape)};
        auto renderShape = std::make_shared<RenderShapeTriangleMesh>(parts);
        newSnode->mesh = renderShape;
        renderShape->setScale({transform.scale.x, transform.scale.y, transform.scale.z});
      }
    }

    for (auto c : node->getChildren()) {
      q.push(c);
      qs.push(newSnode.get());
      qt.push(mat);
    }

    if (parent) {
      parent->children.push_back(std::move(newSnode));
    } else {
      rootNode = std::move(newSnode);
    }
  }

  for (auto l : scene->getPointLights()) {
    auto color = l->getColor();
    auto &T = l->getTransform();
    Pose glPose = {{T.position.x, T.position.y, T.position.z},
                   {T.rotation.w, T.rotation.x, T.rotation.y, T.rotation.z}};
    Pose pose = glPose * POSE_ROS_TO_GL;

    auto c = std::make_shared<SapienRenderPointLightComponent>();
    c->setColor({color.r, color.g, color.b});
    c->setShadowEnabled(true);
    c->setLocalPose({});

    auto newNode = std::make_unique<RenderSceneNode>();
    newNode->pose = pose;
    rootNode->children.push_back(std::move(newNode));
  }

  for (auto l : scene->getDirectionalLights()) {
    auto color = l->getColor();
    auto &T = l->getTransform();
    Pose glPose = {{T.position.x, T.position.y, T.position.z},
                   {T.rotation.w, T.rotation.x, T.rotation.y, T.rotation.z}};
    Pose pose = glPose * POSE_ROS_TO_GL;

    auto c = std::make_shared<SapienRenderDirectionalLightComponent>();
    c->setColor({color.r, color.g, color.b});
    c->setShadowEnabled(true);
    c->setLocalPose({});

    auto newNode = std::make_unique<RenderSceneNode>();
    newNode->pose = pose;
    rootNode->children.push_back(std::move(newNode));
  }

  for (auto l : scene->getSpotLights()) {
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

    auto newNode = std::make_unique<RenderSceneNode>();
    newNode->pose = pose;
    rootNode->children.push_back(std::move(newNode));
  }

  for (auto l : scene->getParallelogramLights()) {
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

    auto newNode = std::make_unique<RenderSceneNode>();
    newNode->pose = pose;
    rootNode->children.push_back(std::move(newNode));
  }

  return rootNode;
}

// std::vector<std::shared_ptr<Entity>> LoadScene(std::string const &filename) {
//   std::vector<std::shared_ptr<Entity>> entities;

//   auto scene = svulkan2::scene::LoadScene(filename);

//   for (auto obj : scene->getObjects()) {
//     auto &T = obj->getTransform();
//     for (auto shape : obj->getModel()->getShapes()) {
//       auto entity = std::make_shared<Entity>();
//       auto body = std::make_shared<SapienRenderBodyComponent>();

//       std::vector<std::shared_ptr<RenderShapeTriangleMeshPart>> parts = {
//           std::make_shared<RenderShapeTriangleMeshPart>(shape)};
//       auto renderShape = std::make_shared<RenderShapeTriangleMesh>(parts);
//       renderShape->setScale({T.scale.x, T.scale.y, T.scale.z});

//       body->setName(shape->name);
//       body->attachRenderShape(renderShape);
//       entity->addComponent(body);

//       entity->setPose({{T.position.x, T.position.y, T.position.z},
//                        {T.rotation.w, T.rotation.x, T.rotation.y, T.rotation.z}});
//       entity->setName("Visual Mesh: " + shape->name);
//       entities.push_back(entity);
//     }
//   }

//   for (auto l : scene->getPointLights()) {
//     auto entity = std::make_shared<Entity>();

//     auto color = l->getColor();
//     auto &T = l->getTransform();
//     Pose glPose = {{T.position.x, T.position.y, T.position.z},
//                    {T.rotation.w, T.rotation.x, T.rotation.y, T.rotation.z}};
//     Pose pose = glPose * POSE_ROS_TO_GL;

//     auto c = std::make_shared<SapienRenderPointLightComponent>();
//     c->setColor({color.r, color.g, color.b});
//     c->setShadowEnabled(true);
//     c->setLocalPose({});

//     entity->addComponent(c);
//     entity->setName("Point Light");
//     entity->setPose(pose);
//     entities.push_back(entity);
//   }

//   for (auto l : scene->getDirectionalLights()) {
//     auto entity = std::make_shared<Entity>();

//     auto color = l->getColor();
//     auto &T = l->getTransform();
//     Pose glPose = {{T.position.x, T.position.y, T.position.z},
//                    {T.rotation.w, T.rotation.x, T.rotation.y, T.rotation.z}};
//     Pose pose = glPose * POSE_ROS_TO_GL;

//     auto c = std::make_shared<SapienRenderDirectionalLightComponent>();
//     c->setColor({color.r, color.g, color.b});
//     c->setShadowEnabled(true);
//     c->setLocalPose({});

//     entity->addComponent(c);
//     entity->setName("Directional Light");
//     entity->setPose(pose);
//     entities.push_back(entity);
//   }

//   for (auto l : scene->getSpotLights()) {
//     auto entity = std::make_shared<Entity>();

//     auto color = l->getColor();
//     auto &T = l->getTransform();
//     Pose glPose = {{T.position.x, T.position.y, T.position.z},
//                    {T.rotation.w, T.rotation.x, T.rotation.y, T.rotation.z}};
//     Pose pose = glPose * POSE_ROS_TO_GL;

//     auto c = std::make_shared<SapienRenderSpotLightComponent>();
//     c->setColor({color.r, color.g, color.b});
//     c->setShadowEnabled(true);
//     c->setFovInner(l->getFovSmall());
//     c->setFovOuter(l->getFov());
//     c->setLocalPose({});

//     entity->addComponent(c);
//     entity->setName("Spot Light");
//     entity->setPose(pose);
//     entities.push_back(entity);
//   }

//   for (auto l : scene->getParallelogramLights()) {
//     auto entity = std::make_shared<Entity>();

//     auto color = l->getColor();
//     auto &T = l->getTransform();
//     Pose glPose = {{T.position.x, T.position.y, T.position.z},
//                    {T.rotation.w, T.rotation.x, T.rotation.y, T.rotation.z}};
//     Pose pose = glPose * POSE_ROS_TO_GL;

//     auto c = std::make_shared<SapienRenderParallelogramLightComponent>();
//     c->setColor({color.r, color.g, color.b});
//     c->setShadowEnabled(true);
//     c->setShape(l->getHalfSize().x, l->getHalfSize().y, l->getAngle());
//     c->setLocalPose({});

//     entity->addComponent(c);
//     entity->setName("Area Light");
//     entity->setPose(pose);
//     entities.push_back(entity);
//   }

//   return entities;
// }

} // namespace sapien_renderer
} // namespace sapien
