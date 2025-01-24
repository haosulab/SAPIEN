/*
 * Copyright 2025 Hillbot Inc.
 * Copyright 2020-2024 UCSD SU Lab
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "sapien/sapien_renderer/sapien_renderer.h"
#include <queue>
#include <svulkan2/scene/loader.h>

namespace sapien {
namespace sapien_renderer {

std::unique_ptr<RenderSceneLoaderNode> LoadScene(std::string const &filename, bool applyScale) {
  auto scene = svulkan2::scene::LoadScene(filename);

  std::unique_ptr<RenderSceneLoaderNode> rootNode;

  std::queue<svulkan2::scene::Node *> q;
  q.push(&scene->getRootNode());

  std::queue<RenderSceneLoaderNode *> qs;
  qs.push(nullptr);

  std::queue<glm::mat4> qt;
  qt.push(glm::mat4(1.f));

  while (!q.empty()) {
    svulkan2::scene::Node *node = q.front();
    q.pop();

    glm::mat4 parentTransform = qt.front();
    qt.pop();

    RenderSceneLoaderNode *parent = qs.front();
    qs.pop();

    auto mat = parentTransform * node->getTransform().matrix();

    auto newSnode = std::make_unique<RenderSceneLoaderNode>();
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

    auto newNode = std::make_unique<RenderSceneLoaderNode>();
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

    auto newNode = std::make_unique<RenderSceneLoaderNode>();
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

    auto newNode = std::make_unique<RenderSceneLoaderNode>();
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

    auto newNode = std::make_unique<RenderSceneLoaderNode>();
    newNode->pose = pose;
    rootNode->children.push_back(std::move(newNode));
  }

  return rootNode;
}

static void _flatten(std::vector<std::shared_ptr<RenderShapeTriangleMesh>> &meshes,
                     std::vector<std::shared_ptr<SapienRenderLightComponent>> &lights,
                     RenderSceneLoaderNode &node, Pose const &parentPose) {
  // TODO: apply scale if not already applied

  Pose pose = parentPose * node.pose;
  node.pose = Pose();

  if (node.mesh) {
    node.mesh->setLocalPose(pose);
    meshes.push_back(node.mesh);
  }
  if (node.light) {
    node.light->setLocalPose(pose);
    lights.push_back(node.light);
  }

  for (auto c : node.children) {
    _flatten(meshes, lights, *c, pose);
  }
}

std::tuple<std::vector<std::shared_ptr<RenderShapeTriangleMesh>>,
           std::vector<std::shared_ptr<SapienRenderLightComponent>>>
RenderSceneLoaderNode::flatten() {
  std::vector<std::shared_ptr<RenderShapeTriangleMesh>> meshes;
  std::vector<std::shared_ptr<SapienRenderLightComponent>> lights;
  _flatten(meshes, lights, *this, Pose());
  return {meshes, lights};
}

} // namespace sapien_renderer
} // namespace sapien
