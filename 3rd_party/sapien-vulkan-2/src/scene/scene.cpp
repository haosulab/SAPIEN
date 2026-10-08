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
#include "svulkan2/scene/scene.h"
#include "../common/logger.h"
#include "svulkan2/core/context.h"
#include <algorithm>

namespace svulkan2 {
namespace scene {

struct PointLightData {
  glm::vec4 position;
  glm::vec4 color;
};

struct DirectionalLightData {
  glm::vec3 direction;
  float softness;
  glm::vec4 color;
};

struct SpotLightData {
  glm::vec4 position;
  glm::vec4 direction;
  glm::vec4 color;
};

struct LightBufferData {
  glm::mat4 viewMatrix;
  glm::mat4 viewMatrixInverse;
  glm::mat4 projectionMatrix;
  glm::mat4 projectionMatrixInverse;
  int width;
  int height;
};

struct ShadowBufferData {
  glm::mat4 viewMatrix;
  glm::mat4 viewMatrixInverse;
  glm::mat4 projectionMatrix;
  glm::mat4 projectionMatrixInverse;
  int width;
  int height;
  int padding[2];
};

Scene::Scene() {
  mNodes.push_back(std::make_unique<Node>());
  mRootNode = mNodes.back().get();
  mRootNode->setScene(this);
}

Node &Scene::addNode(Node &parent, Transform const &transform) {
  updateVersion();
  forceRemove();
  mNodes.push_back(std::make_unique<Node>());
  mNodes.back()->setScene(this);
  mNodes.back()->setParent(parent);
  parent.addChild(*mNodes.back());

  // update global matrix now, so the next call to update the scene
  // will update the current matrix into the previous matrix
  mNodes.back()->setTransform(transform);
  mNodes.back()->updateGlobalModelMatrixRecursive();
  return *mNodes.back();
}

Node &Scene::addNode(Transform const &transform) { return addNode(getRootNode(), transform); }

Object &Scene::addObject(std::shared_ptr<resource::SVModel> model, Transform const &transform) {
  return addObject(getRootNode(), model, transform);
}

Object &Scene::addObject(Node &parent, std::shared_ptr<resource::SVModel> model,
                         Transform const &transform) {
  updateVersion();
  forceRemove();
  auto obj = std::make_unique<Object>(model);
  auto &result = *obj;
  mObjects.push_back(std::move(obj));
  mObjects.back()->setScene(this);
  mObjects.back()->setParent(parent);
  parent.addChild(*mObjects.back());

  mObjects.back()->setTransform(transform);
  mObjects.back()->updateGlobalModelMatrixRecursive();
  return result;
}

Object &Scene::addDeformableObject(std::shared_ptr<resource::SVModel> model) {
  updateVersion();
  forceRemove();
  auto obj = std::make_unique<Object>(model);
  auto &result = *obj;
  mDeformableObjects.push_back(std::move(obj));
  mDeformableObjects.back()->setScene(this);
  mDeformableObjects.back()->setParent(getRootNode());
  getRootNode().addChild(*mDeformableObjects.back());

  mDeformableObjects.back()->setTransform({});
  mDeformableObjects.back()->updateGlobalModelMatrixRecursive();
  return result;
}

LineObject &Scene::addLineObject(std::shared_ptr<resource::SVLineSet> lineSet,
                                 Transform const &transform) {
  return addLineObject(getRootNode(), lineSet, transform);
}

LineObject &Scene::addLineObject(Node &parent, std::shared_ptr<resource::SVLineSet> lineSet,
                                 Transform const &transform) {
  updateVersion();
  forceRemove();
  auto obj = std::make_unique<LineObject>(lineSet);
  auto &result = *obj;
  mLineObjects.push_back(std::move(obj));
  mLineObjects.back()->setScene(this);
  mLineObjects.back()->setParent(parent);
  parent.addChild(*mLineObjects.back());

  mLineObjects.back()->setTransform(transform);
  mLineObjects.back()->updateGlobalModelMatrixRecursive();
  return result;
}

PointObject &Scene::addPointObject(std::shared_ptr<resource::SVPointSet> pointSet,
                                   Transform const &transform) {
  return addPointObject(getRootNode(), pointSet, transform);
}

PointObject &Scene::addPointObject(Node &parent, std::shared_ptr<resource::SVPointSet> pointSet,
                                   Transform const &transform) {
  updateVersion();
  forceRemove();
  auto obj = std::make_unique<PointObject>(pointSet);
  auto &result = *obj;
  mPointObjects.push_back(std::move(obj));
  mPointObjects.back()->setScene(this);
  mPointObjects.back()->setParent(parent);
  parent.addChild(*mPointObjects.back());

  mPointObjects.back()->setTransform(transform);
  mPointObjects.back()->updateGlobalModelMatrixRecursive();
  return result;
}

Camera &Scene::addCamera(Transform const &transform) {
  return addCamera(getRootNode(), transform);
}
Camera &Scene::addCamera(Node &parent, Transform const &transform) {
  updateVersion();
  forceRemove();
  auto cam = std::make_unique<Camera>();
  auto &result = *cam;
  mCameras.push_back(std::move(cam));
  mCameras.back()->setScene(this);
  mCameras.back()->setParent(parent);

  mCameras.back()->setTransform(transform);
  mCameras.back()->updateGlobalModelMatrixRecursive();
  parent.addChild(*mCameras.back());
  return result;
}

PointLight &Scene::addPointLight() { return addPointLight(getRootNode()); }
PointLight &Scene::addPointLight(Node &parent) {
  updateVersion();
  forceRemove();
  auto pointLight = std::make_unique<PointLight>();
  auto &result = *pointLight;
  mPointLights.push_back(std::move(pointLight));
  mPointLights.back()->setScene(this);
  mPointLights.back()->setParent(parent);
  parent.addChild(*mPointLights.back());
  return result;
}

DirectionalLight &Scene::addDirectionalLight() { return addDirectionalLight(getRootNode()); }
DirectionalLight &Scene::addDirectionalLight(Node &parent) {
  updateVersion();
  forceRemove();
  auto directionalLight = std::make_unique<DirectionalLight>();
  auto &result = *directionalLight;
  mDirectionalLights.push_back(std::move(directionalLight));
  mDirectionalLights.back()->setScene(this);
  mDirectionalLights.back()->setParent(parent);
  parent.addChild(*mDirectionalLights.back());
  return result;
}

SpotLight &Scene::addSpotLight() { return addSpotLight(getRootNode()); }
SpotLight &Scene::addSpotLight(Node &parent) {
  updateVersion();
  forceRemove();
  auto spotLight = std::make_unique<SpotLight>();
  auto &result = *spotLight;
  mSpotLights.push_back(std::move(spotLight));
  mSpotLights.back()->setScene(this);
  mSpotLights.back()->setParent(parent);
  parent.addChild(*mSpotLights.back());
  return result;
}

TexturedLight &Scene::addTexturedLight() { return addTexturedLight(getRootNode()); }
TexturedLight &Scene::addTexturedLight(Node &parent) {
  updateVersion();
  forceRemove();
  auto texturedLight = std::make_unique<TexturedLight>();
  auto &result = *texturedLight;
  mTexturedLights.push_back(std::move(texturedLight));
  mTexturedLights.back()->setScene(this);
  mTexturedLights.back()->setParent(parent);
  parent.addChild(*mTexturedLights.back());
  return result;
}

ParallelogramLight &Scene::addParallelogramLight() { return addParallelogramLight(getRootNode()); }
ParallelogramLight &Scene::addParallelogramLight(Node &parent) {
  updateVersion();
  forceRemove();
  auto parallelogramLight = std::make_unique<ParallelogramLight>();
  auto &result = *parallelogramLight;
  mParallelogramLights.push_back(std::move(parallelogramLight));
  mParallelogramLights.back()->setScene(this);
  mParallelogramLights.back()->setParent(parent);
  parent.addChild(*mParallelogramLights.back());
  return result;
}

void Scene::removeNode(Node &node) {
  updateVersion();
  mRequireForceRemove = true;
  node.markRemovedRecursive();
  node.getParent().removeChild(node);
}

void Scene::clearNodes() {
  updateVersion();
  mNodes.resize(1);
  mObjects.clear();
  mDeformableObjects.clear();
  mLineObjects.clear();
  mPointObjects.clear();
  mCameras.clear();
  mPointLights.clear();
  mDirectionalLights.clear();
  mSpotLights.clear();
  mTexturedLights.clear();
  mParallelogramLights.clear();
}

void Scene::forceRemove() {
  if (!mRequireForceRemove) {
    return;
  }
  mNodes.erase(std::remove_if(mNodes.begin(), mNodes.end(),
                              [](std::unique_ptr<Node> &node) { return node->isMarkedRemoved(); }),
               mNodes.end());
  mObjects.erase(
      std::remove_if(mObjects.begin(), mObjects.end(),
                     [](std::unique_ptr<Object> &node) { return node->isMarkedRemoved(); }),
      mObjects.end());
  mDeformableObjects.erase(
      std::remove_if(mDeformableObjects.begin(), mDeformableObjects.end(),
                     [](std::unique_ptr<Object> &node) { return node->isMarkedRemoved(); }),
      mDeformableObjects.end());
  mLineObjects.erase(
      std::remove_if(mLineObjects.begin(), mLineObjects.end(),
                     [](std::unique_ptr<LineObject> &node) { return node->isMarkedRemoved(); }),
      mLineObjects.end());
  mPointObjects.erase(
      std::remove_if(mPointObjects.begin(), mPointObjects.end(),
                     [](std::unique_ptr<PointObject> &node) { return node->isMarkedRemoved(); }),
      mPointObjects.end());
  mCameras.erase(
      std::remove_if(mCameras.begin(), mCameras.end(),
                     [](std::unique_ptr<Camera> &node) { return node->isMarkedRemoved(); }),
      mCameras.end());
  mPointLights.erase(
      std::remove_if(mPointLights.begin(), mPointLights.end(),
                     [](std::unique_ptr<PointLight> &node) { return node->isMarkedRemoved(); }),
      mPointLights.end());
  mDirectionalLights.erase(std::remove_if(mDirectionalLights.begin(), mDirectionalLights.end(),
                                          [](std::unique_ptr<DirectionalLight> &node) {
                                            return node->isMarkedRemoved();
                                          }),
                           mDirectionalLights.end());
  mSpotLights.erase(
      std::remove_if(mSpotLights.begin(), mSpotLights.end(),
                     [](std::unique_ptr<SpotLight> &node) { return node->isMarkedRemoved(); }),
      mSpotLights.end());

  mTexturedLights.erase(
      std::remove_if(mTexturedLights.begin(), mTexturedLights.end(),
                     [](std::unique_ptr<TexturedLight> &node) { return node->isMarkedRemoved(); }),
      mTexturedLights.end());

  mParallelogramLights.erase(std::remove_if(mParallelogramLights.begin(),
                                            mParallelogramLights.end(),
                                            [](std::unique_ptr<ParallelogramLight> &node) {
                                              return node->isMarkedRemoved();
                                            }),
                             mParallelogramLights.end());

  mRequireForceRemove = false;
}

std::vector<Object *> Scene::getObjects() {
  forceRemove();
  std::vector<Object *> result;
  for (auto &obj : mObjects) {
    result.push_back(obj.get());
  }
  for (auto &obj : mDeformableObjects) {
    result.push_back(obj.get());
  }
  return result;
}

std::vector<LineObject *> Scene::getLineObjects() {
  forceRemove();
  std::vector<LineObject *> result;
  for (auto &obj : mLineObjects) {
    result.push_back(obj.get());
  }
  return result;
}

std::vector<PointObject *> Scene::getPointObjects() {
  forceRemove();
  std::vector<PointObject *> result;
  for (auto &obj : mPointObjects) {
    result.push_back(obj.get());
  }
  return result;
}

std::vector<Camera *> Scene::getCameras() {
  forceRemove();
  std::vector<Camera *> result;
  for (auto &obj : mCameras) {
    result.push_back(obj.get());
  }
  return result;
}

std::vector<PointLight *> Scene::getPointLights() {
  forceRemove();
  std::vector<PointLight *> result;
  for (auto &light : mPointLights) {
    result.push_back(light.get());
  }
  return result;
}

std::vector<DirectionalLight *> Scene::getDirectionalLights() {
  forceRemove();
  std::vector<DirectionalLight *> result;
  for (auto &light : mDirectionalLights) {
    result.push_back(light.get());
  }
  return result;
}

std::vector<SpotLight *> Scene::getSpotLights() {
  forceRemove();
  std::vector<SpotLight *> result;
  for (auto &light : mSpotLights) {
    result.push_back(light.get());
  }
  return result;
}

std::vector<TexturedLight *> Scene::getTexturedLights() {
  forceRemove();
  std::vector<TexturedLight *> result;
  for (auto &light : mTexturedLights) {
    result.push_back(light.get());
  }
  return result;
}

std::vector<ParallelogramLight *> Scene::getParallelogramLights() {
  forceRemove();
  std::vector<ParallelogramLight *> result;
  for (auto &light : mParallelogramLights) {
    result.push_back(light.get());
  }
  return result;
}

void Scene::updateModelMatrices() {
  updateRenderVersion();
  mRootNode->updateGlobalModelMatrixRecursive();
}

void Scene::uploadToDevice(core::Buffer &sceneBuffer, StructDataLayout const &sceneLayout) {
  auto pointLights = getPointLights();
  auto directionalLights = getDirectionalLights();
  auto spotLights = getSpotLights();
  auto texturedLights = getTexturedLights();

  std::vector<PointLightData> pointLightData;
  std::vector<DirectionalLightData> directionalLightData;
  std::vector<SpotLightData> spotLightData;
  std::vector<SpotLightData> texturedLightData;
  for (auto light : pointLights) {
    pointLightData.push_back(
        {.position = light->getTransform().worldModelMatrix * glm::vec4(0, 0, 0, 1),
         .color = glm::vec4(light->getColor(), 0)});
  }

  for (auto light : directionalLights) {
    directionalLightData.push_back(
        {.direction = glm::vec3(light->getTransform().worldModelMatrix * glm::vec4(0, 0, -1, 0)),
         .softness = light->getSoftness(),
         .color = glm::vec4(light->getColor(), 0)});
  }

  for (auto light : spotLights) {
    spotLightData.push_back({.position = glm::vec4(light->getPosition(), 1),
                             .direction = glm::vec4(light->getDirection(), light->getFov()),
                             .color = glm::vec4(light->getColor(), light->getFovSmall())});
  }

  for (auto light : texturedLights) {
    texturedLightData.push_back({.position = glm::vec4(light->getPosition(), 1),
                                 .direction = glm::vec4(light->getDirection(), light->getFov()),
                                 .color = glm::vec4(light->getColor(), light->getFovSmall())});
  }

  sceneBuffer.upload(&mAmbientLight, 16, sceneLayout.elements.at("ambientLight").offset);
  uint32_t numPointLights = mPointLights.size();
  uint32_t numDirectionalLights = mDirectionalLights.size();
  uint32_t numSpotLights = mSpotLights.size();
  uint32_t numTexturedLights = mTexturedLights.size();
  uint32_t maxNumPointLights = sceneLayout.elements.at("pointLights").size /
                               sceneLayout.elements.at("pointLights").member->size;
  uint32_t maxNumDirectionalLights = sceneLayout.elements.at("directionalLights").size /
                                     sceneLayout.elements.at("directionalLights").member->size;
  uint32_t maxNumSpotLights = sceneLayout.elements.at("spotLights").size /
                              sceneLayout.elements.at("spotLights").member->size;
  uint32_t maxNumTexturedLights = sceneLayout.elements.at("texturedLights").size /
                                  sceneLayout.elements.at("texturedLights").member->size;

  if (maxNumPointLights < mPointLights.size()) {
    logger::warn("The scene contains more point lights than the maximum number of "
                 "point lights in the shader. Truncated.");
    numPointLights = maxNumPointLights;
  }
  if (maxNumDirectionalLights < mDirectionalLights.size()) {
    logger::warn("The scene contains more directional lights than the maximum number of "
                 "directional lights in the shader. Truncated.");
    numDirectionalLights = maxNumDirectionalLights;
  }
  if (maxNumSpotLights < mSpotLights.size()) {
    logger::warn("The scene contains more spot lights than the maximum number of "
                 "spot lights in the shader. Truncated.");
    numSpotLights = maxNumSpotLights;
  }
  if (maxNumTexturedLights < mTexturedLights.size()) {
    logger::warn("The scene contains more textured lights than the maximum number of "
                 "textured lights in the shader. Truncated.");
    numTexturedLights = maxNumTexturedLights;
  }

  sceneBuffer.upload(pointLightData.data(), numPointLights * sizeof(PointLightData),
                     sceneLayout.elements.at("pointLights").offset);
  sceneBuffer.upload(directionalLightData.data(),
                     numDirectionalLights * sizeof(DirectionalLightData),
                     sceneLayout.elements.at("directionalLights").offset);
  sceneBuffer.upload(spotLightData.data(), numSpotLights * sizeof(SpotLightData),
                     sceneLayout.elements.at("spotLights").offset);
  sceneBuffer.upload(texturedLightData.data(), numTexturedLights * sizeof(SpotLightData),
                     sceneLayout.elements.at("texturedLights").offset);
}

void Scene::uploadShadowToDevice(core::Buffer &shadowBuffer,
                                 std::vector<std::unique_ptr<core::Buffer>> &lightBuffers,
                                 StructDataLayout const &shadowLayout) {

  uint32_t maxNumDirectionalLightShadows =
      shadowLayout.elements.at("directionalLightBuffers").size /
      shadowLayout.elements.at("directionalLightBuffers").member->size;
  uint32_t maxNumPointLightShadows = shadowLayout.elements.at("pointLightBuffers").size /
                                     shadowLayout.elements.at("pointLightBuffers").member->size /
                                     6;
  uint32_t maxNumSpotLightShadows = shadowLayout.elements.at("spotLightBuffers").size /
                                    shadowLayout.elements.at("spotLightBuffers").member->size;
  uint32_t maxNumTexturedLightShadows =
      shadowLayout.elements.at("texturedLightBuffers").size /
      shadowLayout.elements.at("texturedLightBuffers").member->size;

  uint32_t lightBufferIndex = 0;
  {
    std::vector<ShadowBufferData> directionalLightShadowData;
    uint32_t numDirectionalLightShadows = 0;
    for (auto &l : getDirectionalLights()) {
      if (l->isShadowEnabled()) {
        if (numDirectionalLightShadows >= maxNumDirectionalLightShadows) {
          throw std::runtime_error("The scene contains too many directional "
                                   "lights that cast shadows.");
          break;
        }
        numDirectionalLightShadows++;

        auto modelMat = l->getTransform().worldModelMatrix;
        auto projMat = l->getShadowProjectionMatrix();
        directionalLightShadowData.push_back({.viewMatrix = glm::affineInverse(modelMat),
                                              .viewMatrixInverse = modelMat,
                                              .projectionMatrix = projMat,
                                              .projectionMatrixInverse = glm::inverse(projMat),
                                              .width = static_cast<int>(l->getShadowMapSize()),
                                              .height = static_cast<int>(l->getShadowMapSize())});
        lightBuffers[lightBufferIndex++]->upload(&directionalLightShadowData.back(),
                                                 sizeof(LightBufferData));
      } else {
        break;
      }
    }
    shadowBuffer.upload(directionalLightShadowData.data(),
                        directionalLightShadowData.size() * sizeof(ShadowBufferData),
                        shadowLayout.elements.at("directionalLightBuffers").offset);
  }

  {
    std::vector<ShadowBufferData> pointLightShadowData;
    uint32_t numPointLightShadows = 0;
    for (auto &l : getPointLights()) {
      if (l->isShadowEnabled()) {
        if (numPointLightShadows >= maxNumPointLightShadows) {
          throw std::runtime_error("The scene contains too many point "
                                   "lights that cast shadows.");
          // log::warn(
          //     "The scene contains too many point lights that cast shadows. "
          //     "Truncated.");
          break;
        }
        numPointLightShadows++;
        auto modelMats =
            PointLight::getModelMatrices(glm::vec3(l->getTransform().worldModelMatrix[3]));
        auto projMat = l->getShadowProjectionMatrix();
        for (uint32_t i = 0; i < 6; ++i) {
          pointLightShadowData.push_back({.viewMatrix = glm::affineInverse(modelMats[i]),
                                          .viewMatrixInverse = modelMats[i],
                                          .projectionMatrix = projMat,
                                          .projectionMatrixInverse = glm::inverse(projMat),
                                          .width = static_cast<int>(l->getShadowMapSize()),
                                          .height = static_cast<int>(l->getShadowMapSize())});
          lightBuffers[lightBufferIndex++]->upload(&pointLightShadowData.back(),
                                                   sizeof(LightBufferData));
        }
      } else {
        break;
      }
    }
    shadowBuffer.upload(pointLightShadowData.data(),
                        pointLightShadowData.size() * sizeof(ShadowBufferData),
                        shadowLayout.elements.at("pointLightBuffers").offset);
  }

  {
    std::vector<ShadowBufferData> spotLightShadowData;
    uint32_t numSpotLightShadows = 0;
    for (auto &l : getSpotLights()) {
      if (l->isShadowEnabled()) {
        if (numSpotLightShadows >= maxNumSpotLightShadows) {
          throw std::runtime_error("The scene contains too many spot "
                                   "lights that cast shadows.");
          break;
        }
        numSpotLightShadows++;

        auto modelMat = l->getTransform().worldModelMatrix;
        auto projMat = l->getShadowProjectionMatrix();
        spotLightShadowData.push_back({.viewMatrix = glm::affineInverse(modelMat),
                                       .viewMatrixInverse = modelMat,
                                       .projectionMatrix = projMat,
                                       .projectionMatrixInverse = glm::inverse(projMat),
                                       .width = static_cast<int>(l->getShadowMapSize()),
                                       .height = static_cast<int>(l->getShadowMapSize())});
        lightBuffers[lightBufferIndex++]->upload(&spotLightShadowData.back(),
                                                 sizeof(LightBufferData));
      } else {
        break;
      }
    }
    shadowBuffer.upload(spotLightShadowData.data(),
                        spotLightShadowData.size() * sizeof(ShadowBufferData),
                        shadowLayout.elements.at("spotLightBuffers").offset);
  }

  {
    std::vector<ShadowBufferData> texturedLightShadowData;
    uint32_t numTexturedLightShadows = 0;
    for (auto l : getTexturedLights()) {
      if (numTexturedLightShadows >= maxNumTexturedLightShadows) {
        throw std::runtime_error("The scene contains too many textured lights.");
      }
      numTexturedLightShadows++;
      auto modelMat = l->getTransform().worldModelMatrix;
      auto projMat = l->getShadowProjectionMatrix();

      texturedLightShadowData.push_back({.viewMatrix = glm::affineInverse(modelMat),
                                         .viewMatrixInverse = modelMat,
                                         .projectionMatrix = projMat,
                                         .projectionMatrixInverse = glm::inverse(projMat),
                                         .width = static_cast<int>(l->getShadowMapSize()),
                                         .height = static_cast<int>(l->getShadowMapSize())});
      lightBuffers[lightBufferIndex++]->upload(&texturedLightShadowData.back(),
                                               sizeof(LightBufferData));
    }
    shadowBuffer.upload(texturedLightShadowData.data(),
                        texturedLightShadowData.size() * sizeof(ShadowBufferData),
                        shadowLayout.elements.at("texturedLightBuffers").offset);
  }
}

void Scene::reorderLights() {
  updateVersion();
  std::sort(mPointLights.begin(), mPointLights.end(),
            [](auto &l1, auto &l2) { return l1->isShadowEnabled() && !l2->isShadowEnabled(); });
  std::sort(mDirectionalLights.begin(), mDirectionalLights.end(),
            [](auto &l1, auto &l2) { return l1->isShadowEnabled() && !l2->isShadowEnabled(); });
  std::sort(mSpotLights.begin(), mSpotLights.end(),
            [](auto &l1, auto &l2) { return l1->isShadowEnabled() && !l2->isShadowEnabled(); });
}

void Scene::updateVersion() {
  mVersion++;
  mRenderVersion++;
}

void Scene::updateRenderVersion() { mRenderVersion++; }

void Scene::prepareObjectTransformBuffer() {
  size_t gpuTransformBufferSize = getGpuTransformBufferSize();

  // transform buffer is up-to-date
  if (mTransformBufferVersion == mVersion) {
    return;
  }

  // handle back-to-back render, update, render
  core::Context::Get()->getDevice().waitIdle();

  uint32_t count = 0;

  auto objects = getObjects();
  for (auto obj : objects) {
    obj->setInternalGpuIndex(count++);
  }
  auto lineObjects = getLineObjects();
  for (auto obj : lineObjects) {
    obj->setInternalGpuIndex(count++);
  }
  auto pointObjects = getPointObjects();
  for (auto obj : pointObjects) {
    obj->setInternalGpuIndex(count++);
  }

  // no need to create buffer if it is large enough
  if (mTransformBuffer && mTransformBuffer->getSize() >= gpuTransformBufferSize * count) {
    return;
  }

  logger::info("recreating object transform buffer.");

  count = std::max(count, 1u);

  mTransformBufferCpu =
      core::Buffer::Create(gpuTransformBufferSize * count, vk::BufferUsageFlagBits::eTransferSrc,
                           VMA_MEMORY_USAGE_CPU_ONLY);
  mTransformBuffer = core::Buffer::CreateUniform(gpuTransformBufferSize * count, true, true);
  mTransformBufferVersion = mVersion;
}

size_t Scene::getGpuTransformBufferSize() {
  if (!mGpuTransformBufferSize) {
    mGpuTransformBufferSize = std::max(
        (uint64_t)sizeof(glm::mat4),
        (uint64_t)core::Context::Get()->getPhysicalDeviceLimits().minUniformBufferOffsetAlignment);
  }
  return mGpuTransformBufferSize;
}

void Scene::uploadObjectTransforms() {
  if (mTransformBufferRenderVersion == mRenderVersion) {
    return;
  }

  prepareObjectTransformBuffer();

  // collect data
  auto objects = getObjects();
  auto lineObjects = getLineObjects();
  auto pointObjects = getPointObjects();

  size_t totalSize{0};
  uint8_t *buffer = reinterpret_cast<uint8_t *>(mTransformBufferCpu->map());
  size_t step = getGpuTransformBufferSize();
  for (auto obj : objects) {
    auto mat = obj->getTransform().worldModelMatrix;
    std::memcpy(buffer + totalSize, &mat, sizeof(glm::mat4));
    totalSize += step;
  }
  for (auto obj : lineObjects) {
    auto mat = obj->getTransform().worldModelMatrix;
    std::memcpy(buffer + totalSize, &mat, sizeof(glm::mat4));
    totalSize += step;
  }
  for (auto obj : pointObjects) {
    auto mat = obj->getTransform().worldModelMatrix;
    std::memcpy(buffer + totalSize, &mat, sizeof(glm::mat4));
    totalSize += step;
  }
  mTransformBufferCpu->unmap();

  // rendering empty scene
  if (totalSize == 0) {
    return;
  }

  if (!mTransformUpdateCommandBuffer) {
    mTransformUpdateCommandBuffer = getCommandPool().allocateCommandBuffer();
    // mTransformUpdateFence =
    //     core::Context::Get()->getDevice().createFenceUnique({vk::FenceCreateFlagBits::eSignaled});
  }

  // previous frame may still be rendering
  // if (core::Context::Get()->getDevice().waitForFences(mTransformUpdateFence.get(), true,
  //                                                     UINT64_MAX) != vk::Result::eSuccess) {
  //   throw std::runtime_error("failed to wait for fence");
  // }
  // core::Context::Get()->getDevice().resetFences(mTransformUpdateFence.get());

  mTransformUpdateCommandBuffer->reset();
  mTransformUpdateCommandBuffer->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
  {
    // make sure transfer happens after previous reads
    mTransformUpdateCommandBuffer->pipelineBarrier(
        vk::PipelineStageFlagBits::eVertexShader | vk::PipelineStageFlagBits::eFragmentShader,
        vk::PipelineStageFlagBits::eTransfer, {}, {}, {}, {});
  }

  vk::BufferCopy region(0, 0, totalSize);
  mTransformUpdateCommandBuffer->copyBuffer(mTransformBufferCpu->getVulkanBuffer(),
                                            mTransformBuffer->getVulkanBuffer(), region);
  {
    // wait for transfer write to be available, make visible to shader read
    vk::MemoryBarrier barrier(vk::AccessFlagBits::eTransferWrite, vk::AccessFlagBits::eShaderRead);

    // TODO: more shader stages required?
    mTransformUpdateCommandBuffer->pipelineBarrier(vk::PipelineStageFlagBits::eTransfer,
                                                   vk::PipelineStageFlagBits::eVertexShader |
                                                       vk::PipelineStageFlagBits::eFragmentShader,
                                                   {}, barrier, {}, {});
  }

  mTransformUpdateCommandBuffer->end();

  core::Context::Get()->getQueue().submit(mTransformUpdateCommandBuffer.get(), {}
                                          // , mTransformUpdateFence.get()
  );

  mTransformBufferRenderVersion = mRenderVersion;
}

core::CommandPool &Scene::getCommandPool() {
  if (!mCommandPool) {
    mCommandPool = core::Context::Get()->createCommandPool();
  }
  return *mCommandPool;
}

std::shared_ptr<core::Buffer> Scene::getObjectTransformBuffer() {
  prepareObjectTransformBuffer();
  return mTransformBuffer;
}

void Scene::ensureBLAS() {
  // TODO ensure BLAS is not built multiple times, some rigid, some deformable
  for (auto &obj : mObjects) {
    if (!obj->getModel()->getBLAS()) {
      obj->getModel()->buildBLAS(false);
    }
  }
  for (auto &obj : mDeformableObjects) {
    if (!obj->getModel()->getBLAS()) {
      obj->getModel()->buildBLAS(true);
    }
  }
  for (auto &obj : mPointObjects) {
    if (!obj->getPointSet()->getBLAS()) {
      obj->getPointSet()->buildBLAS(true);
    }
  }
}

void Scene::buildTLAS() {
  logger::info("building TLAS");
  auto context = core::Context::Get();
  std::vector<vk::AccelerationStructureInstanceKHR> instances;
  uint32_t instanceIndex{0};

  for (auto obj : getObjects()) {
    glm::mat4 modelTranspose =
        glm::transpose(obj->getTransform().worldModelMatrix); // column major matrix
    vk::TransformMatrixKHR mat;
    static_assert(sizeof(mat) == sizeof(float) * 12);
    std::memcpy(&mat.matrix[0][0], &modelTranspose, sizeof(mat));

    vk::AccelerationStructureInstanceKHR inst(
        mat, instanceIndex, 0xff, 0, vk::GeometryInstanceFlagBitsKHR::eTriangleFacingCullDisable,
        obj->getModel()->getBLAS()->getAddress());

    instanceIndex += obj->getModel()->getShapes().size();

    instances.push_back(inst);
  }

  instanceIndex = 0;
  for (auto obj : getPointObjects()) {
    glm::mat4 modelTranspose = glm::transpose(obj->getTransform().worldModelMatrix);
    vk::TransformMatrixKHR mat;
    std::memcpy(&mat.matrix[0][0], &modelTranspose, sizeof(mat));

    vk::AccelerationStructureInstanceKHR inst(mat, instanceIndex, 0xff, 1, {},
                                              obj->getPointSet()->getBLAS()->getAddress());
    instanceIndex++;
    instances.push_back(inst);
  }

  mTLAS = std::make_unique<core::TLAS>(instances);
  mTLAS->build();
}

void Scene::updateTLAS() {
  if (!mASUpdateCommandBuffer) {
    mASUpdateCommandBuffer = getCommandPool().allocateCommandBuffer();
  }
  mASUpdateCommandBuffer->reset();
  mASUpdateCommandBuffer->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});

  // update BLAS
  for (auto &obj : mDeformableObjects) {
    obj->getModel()->recordUpdateBLAS(mASUpdateCommandBuffer.get());
  }

  // FIXME: update per pointset, not per point cloud
  for (auto &obj : getPointObjects()) {
    obj->getPointSet()->recordUpdateBLAS(mASUpdateCommandBuffer.get());
  }

  std::vector<vk::TransformMatrixKHR> transforms;
  auto objects = getObjects();
  auto pointObjects = getPointObjects();

  transforms.reserve(objects.size() + pointObjects.size());

  for (auto obj : objects) {
    glm::mat4 modelTranspose = glm::transpose(obj->getTransform().worldModelMatrix);
    vk::TransformMatrixKHR mat;
    std::memcpy(&mat.matrix[0][0], &modelTranspose, sizeof(mat));
    transforms.push_back(mat);
  }

  for (auto obj : pointObjects) {
    glm::mat4 modelTranspose = glm::transpose(obj->getTransform().worldModelMatrix);
    vk::TransformMatrixKHR mat;
    std::memcpy(&mat.matrix[0][0], &modelTranspose, sizeof(mat));
    transforms.push_back(mat);
  }

  mTLAS->recordUpdate(mASUpdateCommandBuffer.get(), transforms);
  mASUpdateCommandBuffer->end();
  core::Context::Get()->getQueue().submit(mASUpdateCommandBuffer.get(), {});
}

void Scene::createRTStorageBuffers(StructDataLayout const &materialBufferLayout,
                                   StructDataLayout const &textureIndexBufferLayout,
                                   StructDataLayout const &geometryInstanceBufferLayout) {

  uint32_t geometryInstanceSize = geometryInstanceBufferLayout.size;
  uint32_t materialIndexOffset = geometryInstanceBufferLayout.elements.at("materialIndex").offset;
  uint32_t geometryIndexOffset = geometryInstanceBufferLayout.elements.at("geometryIndex").offset;

  uint32_t textureIndexSize = textureIndexBufferLayout.size;
  uint32_t emissionOffset = textureIndexBufferLayout.elements.at("emission").offset;
  uint32_t diffuseOffset = textureIndexBufferLayout.elements.at("diffuse").offset;
  uint32_t metallicOffset = textureIndexBufferLayout.elements.at("metallic").offset;
  uint32_t roughnessOffset = textureIndexBufferLayout.elements.at("roughness").offset;
  uint32_t normalOffset = textureIndexBufferLayout.elements.at("normal").offset;
  uint32_t transmissionOffset = textureIndexBufferLayout.elements.at("transmission").offset;

  auto objects = getObjects();

  uint32_t instanceCount{0};
  uint32_t meshCount{0};
  uint32_t materialCount{0};
  uint32_t textureCount{0};

  std::unordered_map<std::shared_ptr<resource::SVMesh>, uint32_t> mesh2Id;
  std::unordered_map<std::shared_ptr<resource::SVMaterial>, uint32_t> material2Id;
  std::unordered_map<std::shared_ptr<resource::SVTexture>, uint32_t> texture2Id;

  std::vector<std::shared_ptr<resource::SVMesh>> meshes;
  std::vector<std::shared_ptr<resource::SVMetallicMaterial>> materials;
  std::vector<std::shared_ptr<resource::SVTexture>> textures;

  for (auto &obj : objects) {
    for (auto &shape : obj->getModel()->getShapes()) {
      shape->mesh->uploadToDevice();
      shape->material->uploadToDevice();
      ++instanceCount;

      if (!mesh2Id.contains(shape->mesh)) {
        mesh2Id[shape->mesh] = meshCount++;
        meshes.push_back(shape->mesh);
      }
      if (!material2Id.contains(shape->material)) {
        material2Id[shape->material] = materialCount++;
        auto mat = std::static_pointer_cast<resource::SVMetallicMaterial>(shape->material);
        materials.push_back(mat);

        std::array matTextures{mat->getEmissionTexture(),  mat->getBaseColorTexture(),
                               mat->getNormalTexture(),    mat->getMetallicTexture(),
                               mat->getRoughnessTexture(), mat->getTransmissionTexture()};
        for (auto tex : matTextures) {
          if (tex && !texture2Id.contains(tex)) {
            texture2Id[tex] = textureCount++;
            textures.push_back(tex);
          }
        }
      }
    }
  }

  std::vector<uint8_t> geometryInstanceBuffer(instanceCount * geometryInstanceSize);
  uint32_t instanceIndex = 0;
  for (auto &obj : objects) {
    for (auto &shape : obj->getModel()->getShapes()) {
      uint32_t meshId = mesh2Id.at(shape->mesh);
      uint32_t materialId = material2Id.at(shape->material);
      std::memcpy(geometryInstanceBuffer.data() + geometryInstanceSize * instanceIndex +
                      geometryIndexOffset,
                  &meshId, sizeof(uint32_t));
      std::memcpy(geometryInstanceBuffer.data() + geometryInstanceSize * instanceIndex +
                      materialIndexOffset,
                  &materialId, sizeof(uint32_t));
      ++instanceIndex;
    }
  }

  std::vector<uint8_t> textureIndexBuffer(materialCount * textureIndexSize);
  uint32_t materialIndex{0};
  for (auto mat : materials) {
    auto emissionTex = mat->getEmissionTexture();
    auto diffuseTex = mat->getBaseColorTexture();
    auto normalTex = mat->getNormalTexture();
    auto metallicTex = mat->getMetallicTexture();
    auto roughnessTex = mat->getRoughnessTexture();
    auto transmissionTex = mat->getTransmissionTexture();

    int emissionId = emissionTex ? texture2Id.at(emissionTex) : -1;
    int diffuseId = diffuseTex ? texture2Id.at(diffuseTex) : -1;
    int normalId = normalTex ? texture2Id.at(normalTex) : -1;
    int metallicId = metallicTex ? texture2Id.at(metallicTex) : -1;
    int roughnessId = roughnessTex ? texture2Id.at(roughnessTex) : -1;
    int transmissionId = transmissionTex ? texture2Id.at(transmissionTex) : -1;

    std::memcpy(textureIndexBuffer.data() + materialIndex * textureIndexSize + emissionOffset,
                &emissionId, sizeof(int));
    std::memcpy(textureIndexBuffer.data() + materialIndex * textureIndexSize + diffuseOffset,
                &diffuseId, sizeof(int));
    std::memcpy(textureIndexBuffer.data() + materialIndex * textureIndexSize + normalOffset,
                &normalId, sizeof(int));
    std::memcpy(textureIndexBuffer.data() + materialIndex * textureIndexSize + metallicOffset,
                &metallicId, sizeof(int));
    std::memcpy(textureIndexBuffer.data() + materialIndex * textureIndexSize + roughnessOffset,
                &roughnessId, sizeof(int));
    std::memcpy(textureIndexBuffer.data() + materialIndex * textureIndexSize + transmissionOffset,
                &transmissionId, sizeof(int));

    ++materialIndex;
  }

  mVertexBuffers.clear();
  mIndexBuffers.clear();
  for (auto mesh : meshes) {
    mVertexBuffers.push_back(mesh->getVertexBuffer().getVulkanBuffer());
    mIndexBuffers.push_back(mesh->getIndexBuffer().getVulkanBuffer());
  }

  // build materials
  mMaterialBuffers.clear();
  for (auto mat : materials) {
    mMaterialBuffers.push_back(mat->getDeviceBuffer().getVulkanBuffer());
  }

  // build texture index
  mTextureIndexBuffer = core::Buffer::Create(std::max(size_t(1), textureIndexBuffer.size()),
                                             vk::BufferUsageFlagBits::eStorageBuffer |
                                                 vk::BufferUsageFlagBits::eTransferDst,
                                             VMA_MEMORY_USAGE_GPU_ONLY);
  mTextureIndexBuffer->upload(textureIndexBuffer);

  mGeometryInstanceBuffer = core::Buffer::Create(
      std::max(size_t(1), geometryInstanceBuffer.size()),
      vk::BufferUsageFlagBits::eStorageBuffer | vk::BufferUsageFlagBits::eTransferDst,
      VMA_MEMORY_USAGE_GPU_ONLY);
  mGeometryInstanceBuffer->upload(geometryInstanceBuffer);

  // point objects
  uint32_t pointsetCount{0};
  auto pointObjects = getPointObjects();
  std::unordered_map<std::shared_ptr<resource::SVPointSet>, uint32_t> pointset2Id;
  std::vector<std::shared_ptr<resource::SVPointSet>> pointsets;
  std::vector<int> pointInstanceBuffer;
  for (auto &obj : pointObjects) {
    if (!pointset2Id.contains(obj->getPointSet())) {
      pointset2Id[obj->getPointSet()] = pointsetCount++;
      pointsets.push_back(obj->getPointSet());
    }
    pointInstanceBuffer.push_back(pointset2Id[obj->getPointSet()]);
  }
  mPointInstanceBuffer = core::Buffer::Create(
      std::max(size_t(1), pointInstanceBuffer.size() * sizeof(int)),
      vk::BufferUsageFlagBits::eStorageBuffer | vk::BufferUsageFlagBits::eTransferDst,
      VMA_MEMORY_USAGE_GPU_ONLY);
  mPointInstanceBuffer->upload(pointInstanceBuffer);

  // point sets
  mPointSetBuffers.clear();
  for (auto s : pointsets) {
    mPointSetBuffers.push_back(s->getVertexBuffer().getVulkanBuffer());
  }

  // begin lights
  mRTPointLightBufferHost.clear();
  for (auto &l : mPointLights) {
    mRTPointLightBufferHost.push_back(
        RTPointLight{.position = l->getPosition(), .radius = 0, .rgb = l->getColor()});
  }
  mRTPointLightBuffer = core::Buffer::Create(
      std::max(getPointLights().size(), size_t(1)) * sizeof(RTPointLight),
      vk::BufferUsageFlagBits::eStorageBuffer | vk::BufferUsageFlagBits::eTransferDst,
      VMA_MEMORY_USAGE_CPU_TO_GPU);
  mRTPointLightBuffer->upload(mRTPointLightBufferHost);

  mRTDirectionalLightBufferHost.clear();
  for (auto &l : mDirectionalLights) {
    mRTDirectionalLightBufferHost.push_back(
        RTDirectionalLight{.direction = l->getDirection(), .softness = 0, .rgb = l->getColor()});
  }
  mRTDirectionalLightBuffer = core::Buffer::Create(
      std::max(getDirectionalLights().size(), size_t(1)) * sizeof(RTDirectionalLight),
      vk::BufferUsageFlagBits::eStorageBuffer | vk::BufferUsageFlagBits::eTransferDst,
      VMA_MEMORY_USAGE_CPU_TO_GPU);
  mRTDirectionalLightBuffer->upload(mRTDirectionalLightBufferHost);

  mRTSpotLightBufferHost.clear();
  for (auto &l : mSpotLights) {
    mRTSpotLightBufferHost.push_back(
        RTSpotLight{.viewMat = glm::affineInverse(l->getTransform().worldModelMatrix),
                    .projMat = l->getShadowProjectionMatrix(),
                    .rgb = l->getColor(),
                    .position = l->getPosition(),
                    .fovInner = l->getFovSmall(),
                    .fovOuter = l->getFov(),
                    .textureId = -1});
  }
  for (auto &l : mTexturedLights) {
    auto tex = l->getTexture();
    tex->loadAsync().get(); // TODO: move to top
    tex->uploadToDevice();
    if (tex && !texture2Id.contains(tex)) {
      textures.push_back(tex);
      texture2Id[tex] = textures.size() - 1;
    }
    int textureId = texture2Id.at(tex);

    mRTSpotLightBufferHost.push_back(
        RTSpotLight{.viewMat = glm::affineInverse(l->getTransform().worldModelMatrix),
                    .projMat = l->getShadowProjectionMatrix(),
                    .rgb = l->getColor(),
                    .position = l->getPosition(),
                    .fovInner = l->getFovSmall(),
                    .fovOuter = l->getFov(),
                    .textureId = textureId});
  }
  mRTSpotLightBuffer = core::Buffer::Create(
      std::max(getTexturedLights().size() + getSpotLights().size(), size_t(1)) *
          sizeof(RTSpotLight),
      vk::BufferUsageFlagBits::eStorageBuffer | vk::BufferUsageFlagBits::eTransferDst,
      VMA_MEMORY_USAGE_CPU_TO_GPU);
  mRTSpotLightBuffer->upload(mRTSpotLightBufferHost);

  mRTParallelogramLightBufferHost.clear();
  for (auto &l : mParallelogramLights) {
    mRTParallelogramLightBufferHost.push_back(RTParallelogramLight{
        .color = l->getColor(),
        .position = l->getOrigin(),
        .edge0 = l->getEdgeX(),
        .edge1 = l->getEdgeY(),
    });
  }
  mRTParallelogramLightBuffer = core::Buffer::Create(
      std::max(getParallelogramLights().size(), size_t(1)) * sizeof(RTParallelogramLight),
      vk::BufferUsageFlagBits::eStorageBuffer | vk::BufferUsageFlagBits::eTransferDst,
      VMA_MEMORY_USAGE_CPU_TO_GPU);
  mRTParallelogramLightBuffer->upload(mRTParallelogramLightBufferHost);
  // end lights

  // build textures
  mTextures.clear();
  for (auto texture : textures) {
    mTextures.push_back({texture->getImageView(), texture->getSampler()});
  }
}

void Scene::updateRTStorageBuffers() {
  uint32_t i = 0;
  for (auto &l : mPointLights) {
    auto &b = mRTPointLightBufferHost.at(i);
    b.position = l->getPosition();
    b.radius = 0;
    b.rgb = l->getColor();
    ++i;
  }
  mRTPointLightBuffer->upload(mRTPointLightBufferHost);

  i = 0;
  for (auto &l : mDirectionalLights) {
    auto &b = mRTDirectionalLightBufferHost.at(i);
    b.direction = l->getDirection();
    b.softness = 0;
    b.rgb = l->getColor();
    ++i;
  }
  mRTDirectionalLightBuffer->upload(mRTDirectionalLightBufferHost);

  i = 0;
  for (auto &l : mSpotLights) {
    auto &b = mRTSpotLightBufferHost.at(i);
    b.viewMat = glm::affineInverse(l->getTransform().worldModelMatrix);
    b.projMat = l->getShadowProjectionMatrix();
    b.rgb = l->getColor();
    b.position = l->getPosition();
    b.fovInner = l->getFovSmall();
    b.fovOuter = l->getFov();
    ++i;
  }
  for (auto &l : mTexturedLights) {
    auto &b = mRTSpotLightBufferHost.at(i);
    b.viewMat = glm::affineInverse(l->getTransform().worldModelMatrix);
    b.projMat = l->getShadowProjectionMatrix();
    b.rgb = l->getColor();
    b.position = l->getPosition();
    b.fovInner = l->getFovSmall();
    b.fovOuter = l->getFov();
    ++i;
  }
  mRTSpotLightBuffer->upload(mRTSpotLightBufferHost);

  i = 0;
  for (auto &l : mParallelogramLights) {
    auto &b = mRTParallelogramLightBufferHost.at(i);
    b.color = l->getColor();
    b.position = l->getOrigin();
    b.edge0 = l->getEdgeX();
    b.edge1 = l->getEdgeY();
    ++i;
  }
  mRTParallelogramLightBuffer->upload(mRTParallelogramLightBufferHost);
}

void Scene::buildRTResources(StructDataLayout const &materialBufferLayout,
                             StructDataLayout const &textureIndexBufferLayout,
                             StructDataLayout const &geometryInstanceBufferLayout) {

  std::lock_guard lock(mRTResourcesLock);

  // make sure scene is not in use
  if (mAccessFences.size()) {
    if (core::Context::Get()->getDevice().waitForFences(mAccessFences, VK_TRUE, UINT64_MAX) !=
        vk::Result::eSuccess) {
      throw std::runtime_error("failed to wait for scene access fence");
    }
  }

  forceRemove();

  if (mRTResourcesVersion != mVersion) {
    ensureBLAS();
    buildTLAS();
    createRTStorageBuffers(materialBufferLayout, textureIndexBufferLayout,
                           geometryInstanceBufferLayout);
    mRTResourcesVersion = mVersion;
  }
}

void Scene::updateRTResources() {
  std::lock_guard lock(mRTResourcesLock);
  if (mRTResourcesVersion != mVersion) {
    throw std::runtime_error("updateRTResources failed: scene has changed, "
                             "call buildRTResources first");
  }

  // make sure scene is not in use
  if (mAccessFences.size()) {
    if (core::Context::Get()->getDevice().waitForFences(mAccessFences, VK_TRUE, UINT64_MAX) !=
        vk::Result::eSuccess) {
      throw std::runtime_error("failed to wait for scene access fence");
    }
  }

  updateTLAS();
  updateRTStorageBuffers();
  mRTResourcesRenderVersion = mRenderVersion;
}

void Scene::registerAccessFence(vk::Fence fence) { mAccessFences.push_back(fence); }
void Scene::unregisterAccessFence(vk::Fence fence) { std::erase(mAccessFences, fence); }

} // namespace scene
} // namespace svulkan2
